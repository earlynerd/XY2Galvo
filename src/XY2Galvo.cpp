#include "XY2Galvo.h"
#include <cstring>

// --- Global state for the library ---
LaserQueue laser_queue;
Point XY2Galvo::pos0;
Transformation XY2Galvo::transformation0;
Transformation XY2Galvo::transformation1;
Transformation XY2Galvo::transformation_stack[8];
uint XY2Galvo::transformation_stack_index = 0;

static volatile bool core1_running = false;

// Pre-defined laser settings with speed in units per second.
LaserSet laser_set[] =
{
	{.speed=MAX_SPEED_UPS,        .pattern=0x003, .delay_a=0, .delay_m=0, .delay_e=20}, // 0: jump (laser off pattern)
	{.speed=MAX_SPEED_UPS * 0.8f, .pattern=0x3FF, .delay_a=10, .delay_m=6, .delay_e=10},  // 1: fast straight
	{.speed=MAX_SPEED_UPS * 0.4f, .pattern=0x3FF, .delay_a=10, .delay_m=6, .delay_e=10},  // 2: slow straight
	{.speed=MAX_SPEED_UPS * 0.8f, .pattern=0x3FF, .delay_a=10, .delay_m=0, .delay_e=10},  // 3: fast rounded
	{.speed=MAX_SPEED_UPS * 0.4f, .pattern=0x3FF, .delay_a=10, .delay_m=0, .delay_e=10},  // 4: slow rounded
    {.speed=50000.0f,             .pattern=0x3FF, .delay_a=10, .delay_m=0, .delay_e=10},  // 5: very slow setting
};


// --- LaserQueue Implementation ---
void LaserQueue::push (Data32 data) {
    while (!free()) { tight_loop_contents(); } // Wait for space
    putc(data);
}
void LaserQueue::push (const Point& p) { push(p.x); push(p.y); }
void LaserQueue::push (const Rect& r) { push(r.top_left()); push(r.bottom_right()); }
Data32 LaserQueue::pop () {
    while (!avail()) { tight_loop_contents(); } // Wait for data
    return getc();
}
Point LaserQueue::pop_Point () {
    float x = pop().f;
    float y = pop().f;
    return Point(x,y);
}
Rect LaserQueue::pop_Rect () {
    Point p1 = pop_Point();
    Point p2 = pop_Point();
    return Rect(p1,p2);
}

// --- XY2Galvo Class Implementation ---

XY2Galvo::XY2Galvo() {}

void XY2Galvo::init() {
    uint mask  = (1u<<PIN_XY2_LASER)|(1u<<PIN_XY2_SYNC_XY)|(3u<<PIN_XY2_CLOCK)|(3u<<PIN_XY2_SYNC)|(3u<<PIN_XY2_X)|(3u<<PIN_XY2_Y);
	pio_sm_set_pins_with_mask(PIO_XY2, sm_x, ~(1u<<PIN_XY2_LASER), mask);
	pio_sm_set_pindirs_with_mask(PIO_XY2, sm_x, -1u, mask);
	for(uint i=0;i<32;i++) { if (mask & (1u<<i)) pio_gpio_init(PIO_XY2, i); }

	// Init PIO state machines
	uint offset = pio_add_program(PIO_XY2, &xy2_clock_program);
	pio_sm_config c = xy2_clock_program_get_default_config(offset);
	sm_config_set_sideset_pins(&c, PIN_XY2_CLOCK);
	sm_config_set_clkdiv(&c, float(F_CPU) / XY2_SM_CLOCK);
	pio_sm_init(PIO_XY2, sm_clock, offset + xy2_clock_offset_start, &c);

	offset = pio_add_program(PIO_XY2, &xy2_data_program);
	c = xy2_data_program_get_default_config(offset);
	sm_config_set_clkdiv(&c, float(F_CPU) / XY2_SM_CLOCK);
	sm_config_set_out_shift(&c, false, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	sm_config_set_jmp_pin(&c, PIN_XY2_SYNC_XY);

	sm_config_set_sideset_pins(&c, PIN_XY2_X);
	pio_sm_init(PIO_XY2, sm_x, offset + xy2_data_offset_start, &c);

	sm_config_set_sideset_pins(&c, PIN_XY2_Y);
	pio_sm_init(PIO_XY2, sm_y, offset + xy2_data_offset_start, &c);

	offset = pio_add_program(PIO_XY2, &xy2_laser_program);
	c = xy2_laser_program_get_default_config(offset);
	sm_config_set_clkdiv(&c, float(F_CPU) / XY2_SM_CLOCK);
	sm_config_set_out_shift(&c, true, false, 10);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);
	sm_config_set_out_pins(&c, PIN_XY2_LASER, 1);
	sm_config_set_sideset_pins(&c, PIN_XY2_SYNC_XY);
	pio_sm_init(PIO_XY2, sm_laser, offset + xy2_laser_offset_start, &c);

    // --- LASER POLARITY CONTROL ---
	// To invert the laser signal (active-low), uncomment the following line:
	// gpio_set_outover(PIN_XY2_LASER, GPIO_OVERRIDE_INVERT);
}

void XY2Galvo::start() {
    if (!core1_running) {
        multicore_launch_core1(XY2Galvo::worker);
    }
    while (!core1_running) { tight_loop_contents(); }
}

__attribute__((noreturn)) void XY2Galvo::worker() {
    core1_running = true;

    pio_sm_set_enabled(PIO_XY2, sm_laser, false);
    pio_sm_set_enabled(PIO_XY2, sm_clock, false);
    pio_sm_set_enabled(PIO_XY2, sm_x, false);
    pio_sm_set_enabled(PIO_XY2, sm_y, false);

	pio_sm_clear_fifos(PIO_XY2, sm_laser);
	pio_sm_clear_fifos(PIO_XY2, sm_x);
	pio_sm_clear_fifos(PIO_XY2, sm_y);

	pio_enable_sm_mask_in_sync(PIO_XY2, (1u<<sm_clock)|(1u<<sm_x)|(1u<<sm_y)|(1u<<sm_laser));

    memset(&laser_queue, 0, sizeof(laser_queue));
    pio_send_data(0.0f, 0.0f, 0x000); // Start at center with laser off

    for(;;) {
        DrawCmd cmd = laser_queue.pop().cmd;
        switch(cmd) {
            case CMD_MOVETO: {
                Point p1 = laser_queue.pop_Point();
                execute_lineto(p1, laser_set[0].pattern); // JUMP uses pattern from laser_set 0
                break;
            }
            case CMD_LINETO: {
			    uint pattern = laser_queue.pop().u;
			    Point p1 = laser_queue.pop_Point();
			    execute_lineto(p1, pattern);
                break;
            }
            case CMD_DWELL: {
                uint32_t steps = laser_queue.pop().u;
                uint32_t pattern = laser_queue.pop().u;
                for (uint32_t i = 0; i < steps; ++i) {
                    send_data_blocking(pos0, pattern); // Resend last position to keep laser state
                }
                break;
            }
            case CMD_RESET_TRANSFORMATION: {
                transformation1.reset();
                break;
            }
            case CMD_SET_TRANSFORMATION: {
                while (laser_queue.avail()<6) {}
                Data32* dest = reinterpret_cast<Data32*>(&transformation1);
                laser_queue.read(dest,6);
                transformation1.is_projected = false;
                break;
            }
            case CMD_SET_TRANSFORMATION_3D: {
                while (laser_queue.avail()<9) {}
                Data32* dest = reinterpret_cast<Data32*>(&transformation1);
                laser_queue.read(dest,9);
                transformation1.is_projected = true;
                break;
            }
            case CMD_END:
            default:
                break;
        }
    }
}


// --- Core 1 (Worker) Logic ---

static uint laser_delay_queue[16];
static uint laser_delay_size = 14;
static uint laser_delay_index = 0;

uint XY2Galvo::delayed_laser_value (uint value) {
	if (laser_delay_index >= laser_delay_size) laser_delay_index = 0;
	std::swap(value, laser_delay_queue[laser_delay_index++]);
	return value;
}

void XY2Galvo::pio_wait_free() {
    while (pio_sm_is_tx_fifo_full(PIO_XY2, sm_x)) { tight_loop_contents(); }
}

void XY2Galvo::pio_send_data (float x, float y, uint32_t laser) {
    pos0.x = x;
    pos0.y = y;
    laser = delayed_laser_value(laser);

    uint32_t ix = 0x8000 + int32_t(x);
    uint32_t iy = 0x8000 - int32_t(y);
    if (ix>>16) ix = int32_t(ix)<0 ? 0 : 0xffff;
    if (iy>>16) iy = int32_t(iy)<0 ? 0 : 0xffff;

    pio_sm_put_blocking(PIO_XY2, sm_x, ix);
    pio_sm_put_blocking(PIO_XY2, sm_y, iy);
    pio_sm_put_blocking(PIO_XY2, sm_laser, laser);
}

void XY2Galvo::send_data_blocking (const Point& p, uint32_t laser) {
    pio_wait_free();
    pio_send_data(p.x, p.y, laser);
}

void __not_in_flash_func(XY2Galvo::execute_lineto) (Point dest, uint pattern) {
	transformation1.transform(dest);
	Dist dist_vec = dest - pos0;
	float line_length = dist_vec.length();
    if (line_length <= 0.001f) {
        // For a zero-length move, just send the final position once.
        send_data_blocking(dest, pattern);
        return;
    }

    // This function now *always* runs at max hardware speed.
    // It breaks the line into the smallest possible steps the hardware can manage.
    const float step_speed = SCANNER_MAX_STEP_SPEED;
	Dist step = dist_vec * (step_speed / line_length);

    while(line_length > step_speed) {
        line_length -= step_speed;
        pos0 = pos0 + step;
        send_data_blocking(pos0, pattern);
    }

    // Final step to the exact destination
    if (pos0 != dest) {
        send_data_blocking(dest, pattern);
    }
}


// --- Core 0 (Public API) Logic ---

// Internal helper to queue a dwell command
static void dwell(uint32_t microseconds, uint pattern) {
    if (microseconds == 0) return;
    uint32_t steps = microseconds / 10;
    if (steps == 0) return;
    Data32 s;
    s.u = steps;
    laser_queue.push(CMD_DWELL);
    laser_queue.push(s);
    laser_queue.push(pattern);
}

// Internal helper to plan a single line segment
static void plan_line(const Point& start, const Point& end, const LaserSet& set) {
    Dist dist_vec = end - start;
    float total_dist = dist_vec.length();
    
    // If it's a zero-length line, just send the command.
    if (total_dist < 0.001f) {
        laser_queue.push(CMD_LINETO);
        laser_queue.push(set.pattern);
        laser_queue.push(end);
        return;
    }

    float target_speed_ups = min(set.speed, MAX_SPEED_UPS);
    if (target_speed_ups < 1.0f) target_speed_ups = 1.0f;

    float total_duration_s = total_dist / target_speed_ups;
    constexpr float TARGET_SEGMENT_DURATION_S = 1.0f / TARGET_UPDATE_RATE_HZ;
    uint num_segments = max(1u, (uint)ceilf(total_duration_s / TARGET_SEGMENT_DURATION_S));
    Dist segment_vec = dist_vec / num_segments;
    float time_per_segment_s = total_duration_s / num_segments;
    float time_for_segment_at_max_speed_s = segment_vec.length() / MAX_SPEED_UPS;

    uint32_t delay_per_segment_us = 0;
    if (time_per_segment_s > time_for_segment_at_max_speed_s) {
        delay_per_segment_us = (uint32_t)((time_per_segment_s - time_for_segment_at_max_speed_s) * 1000000.0f);
    }
    
    // --- Queue up commands ---
    Point current_pos = start;
    for (uint i = 0; i < num_segments; ++i) {
        current_pos = (i == num_segments - 1) ? end : current_pos + segment_vec;
        
        laser_queue.push(CMD_LINETO);
        laser_queue.push(set.pattern);
        laser_queue.push(current_pos);

        if (delay_per_segment_us > 0 && i < num_segments - 1) {
            dwell(delay_per_segment_us, set.pattern);
        }
    }
}

void XY2Galvo::moveTo (const Point& p) {
	laser_queue.push(CMD_MOVETO);
	laser_queue.push(p);
}

void XY2Galvo::drawLine (const Point& start, const Point& end, const LaserSet& set) {
	moveTo(start);
    dwell(set.delay_a * 10, laser_set[0].pattern); // Start delay is laser-off
    plan_line(start, end, set);
    dwell(set.delay_e * 10, set.pattern); // End delay is laser-on
}


void XY2Galvo::drawRect (const Rect& rect, const LaserSet& set) {
	Point p1 = rect.top_left();
    Point p2 = rect.top_right();
    Point p3 = rect.bottom_right();
    Point p4 = rect.bottom_left();

    moveTo(p1); 
    dwell(set.delay_a * 10, laser_set[0].pattern);

    plan_line(p1, p2, set);
    dwell(set.delay_m * 10, set.pattern);
    plan_line(p2, p3, set);
    dwell(set.delay_m * 10, set.pattern);
    plan_line(p3, p4, set);
    dwell(set.delay_m * 10, set.pattern);
    plan_line(p4, p1, set);

    dwell(set.delay_e * 10, set.pattern);
}

void XY2Galvo::drawPolyLine (uint count, const Point points[], const LaserSet& set, PolyLineOptions flags) {
	if (count < 1) return;

    if (count == 1) {
        moveTo(points[0]);
        dwell(set.delay_a * 10, laser_set[0].pattern);
        plan_line(points[0], points[0], set); // Dwell at the point
        dwell(set.delay_e * 10, set.pattern);
        return;
    }

    moveTo(points[0]);
    dwell(set.delay_a * 10, laser_set[0].pattern);

    for (uint i = 0; i < count - 1; i++) {
        plan_line(points[i], points[i+1], set);
        
        bool is_last_segment = (i == count - 2);
        if (!is_last_segment || flags == POLYLINE_CLOSED) {
            dwell(set.delay_m * 10, set.pattern);
        }
    }

    if (flags == POLYLINE_CLOSED) {
        plan_line(points[count-1], points[0], set);
    }

    dwell(set.delay_e * 10, set.pattern);
}

void XY2Galvo::drawPolygon (uint count, const Point points[], const LaserSet& set) {
	drawPolyLine(count, points, set, POLYLINE_CLOSED);
}

void XY2Galvo::wait(uint32_t microseconds) {
    dwell(microseconds, laser_set[0].pattern); // Public wait is always laser-off
}

void XY2Galvo::update_transformation() {
    static_assert(sizeof(Data32) == sizeof(float), "Size mismatch");
    laser_queue.push(transformation0.is_projected ? CMD_SET_TRANSFORMATION_3D : CMD_SET_TRANSFORMATION);
    uint n = transformation0.is_projected ? 9 : 6;
    while (laser_queue.free() < n) { tight_loop_contents(); }
    Data32* data = reinterpret_cast<Data32*>(&transformation0);
    laser_queue.write(data, n);
}

void XY2Galvo::resetTransformation() {
    transformation0.reset();
    laser_queue.push(CMD_RESET_TRANSFORMATION);
}

void XY2Galvo::setTransformation (const Transformation& t) {
    transformation0 = t;
    update_transformation();
}

void XY2Galvo::pushTransformation() {
    transformation_stack[--transformation_stack_index & 7] = transformation0;
}

void XY2Galvo::popTransformation() {
    transformation0 = transformation_stack[transformation_stack_index++ & 7];
    update_transformation();
}
void XY2Galvo::rotate(float rad) {
    transformation0.rotate(rad);
    update_transformation();
}
void XY2Galvo::scale(float s) {
    transformation0.scale(s);
    update_transformation();
}
void XY2Galvo::scale(float sx, float sy) {
    transformation0.scale(sx, sy);
    update_transformation();
}
void XY2Galvo::addOffset(float dx, float dy) {
    transformation0.addOffset(dx, dy);
    update_transformation();
}

