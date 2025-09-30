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
static volatile bool core1_suspend = false;
static volatile bool core1_suspended = false;

static uint laser_delay_queue[16] = {0};
static uint laser_delay_size = 14; // From LASER_QUEUE_DELAY
static uint laser_delay_index = 0;

// Pre-defined laser settings
LaserSet laser_set[] =
{
	{.speed=SCANNER_MAX_SPEED, .pattern=0x003, .delay_a=0, .delay_m=0, .delay_e=20},	// 0: jump
	{.speed=SCANNER_MAX_SPEED, .pattern=0x3FF, .delay_a=0, .delay_m=6, .delay_e=0},	// 1: fast straight
	{.speed=SCANNER_MAX_SPEED*2/3, .pattern=0x3FF, .delay_a=0, .delay_m=6, .delay_e=0},	// 2: slow straight
	{.speed=SCANNER_MAX_SPEED, .pattern=0x3FF, .delay_a=0, .delay_m=0, .delay_e=0},	// 3: fast rounded
	{.speed=SCANNER_MAX_SPEED*2/3, .pattern=0x3FF, .delay_a=0, .delay_m=0, .delay_e=0},	// 4: slow rounded
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
    FLOAT x = pop().f;
    FLOAT y = pop().f;
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

	gpio_set_outover(PIN_XY2_LASER, GPIO_OVERRIDE_INVERT);
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

    memset(laser_delay_queue, 0, sizeof(laser_delay_queue));
    pio_send_data(0.0f, 0.0f, 0x000);

    for(;;) {
        DrawCmd cmd = laser_queue.pop().cmd;
        switch(cmd) {
            case CMD_MOVETO: {
                Point p1 = laser_queue.pop_Point();
                move_to(p1);
                break;
            }
            case CMD_DRAWTO: {
                const LaserSet* set = laser_queue.pop().set;
                Point p1 = laser_queue.pop_Point();
                uint laser_on_delay=0;
                draw_to(p1,set->speed,set->pattern,laser_on_delay,set->delay_m);
                break;
            }
            case CMD_LINETO: {
			    const LaserSet* set = laser_queue.pop().set;
			    Point p1 = laser_queue.pop_Point();
			    line_to(p1,*set);
                break;
            }
            case CMD_LINE: {
                const LaserSet* set = laser_queue.pop().set;
                Point p1 = laser_queue.pop_Point();
                Point p2 = laser_queue.pop_Point();
                draw_line(p1,p2,*set);
                break;
            }
            case CMD_RECT: {
                const LaserSet* set = laser_queue.pop().set;
                Rect rect = laser_queue.pop_Rect();
                draw_rect(rect,*set);
                break;
            }
            case CMD_POLYLINE: {
                const LaserSet* set = laser_queue.pop().set;
                uint flags = laser_queue.pop().u;
                uint count = laser_queue.pop().u;
                draw_polyline(count, [](){return laser_queue.pop_Point();}, *set, flags);
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
                // Do nothing, loop
                break;
        }
    }
}


// --- Core Drawing Logic (runs on Core 1) ---

uint XY2Galvo::delayed_laser_value (uint value) {
	if (laser_delay_index >= laser_delay_size) laser_delay_index = 0;
	std::swap(value, laser_delay_queue[laser_delay_index++]);
	return value;
}

void XY2Galvo::pio_wait_free() {
    while (pio_sm_is_tx_fifo_full(PIO_XY2, sm_x)) { tight_loop_contents(); }
}

void XY2Galvo::pio_send_data (FLOAT x, FLOAT y, uint32_t laser) {
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


void __not_in_flash_func(XY2Galvo::draw_to) (Point dest, FLOAT speed, uint laser_on_pattern, uint& laser_on_delay, uint end_delay) {
	transformation1.transform(dest);
	Dist dist = dest - pos0;
	FLOAT line_length = dist.length();
	if (line_length == 0) return;
    Dist step = dist * (speed / line_length);

	uint laser_off_pattern = laser_set[0].pattern;

    while(line_length > speed) {
        line_length -= speed;
        uint current_pattern = (laser_on_delay > 0) ? laser_off_pattern : laser_on_pattern;
        send_data_blocking(pos0 + step, current_pattern);
        if (laser_on_delay > 0) laser_on_delay--;
    }

    if (pos0 != dest) {
        uint current_pattern = (laser_on_delay > 0) ? laser_off_pattern : laser_on_pattern;
        send_data_blocking(dest, current_pattern);
        if (laser_on_delay > 0) laser_on_delay--;
    }

    for (uint i = 0; i < end_delay; i++) {
        uint current_pattern = (laser_on_delay > 0) ? laser_off_pattern : laser_on_pattern;
        send_data_blocking(dest, current_pattern);
        if (laser_on_delay > 0) laser_on_delay--;
    }
}


void XY2Galvo::move_to (const Point& dest) { line_to(dest, laser_set[0]); }

void XY2Galvo::line_to (const Point& dest, const LaserSet& set) {
	uint laser_on_delay = set.delay_a;
	draw_to(dest, set.speed, set.pattern, laser_on_delay, set.delay_e);
}

void XY2Galvo::draw_line (const Point& start, const Point& dest, const LaserSet& set) {
	move_to(start);
	line_to(dest, set);
}

void XY2Galvo::draw_rect (const Rect& bbox, const LaserSet& set) {
	move_to(bbox.top_left());
	line_to(bbox.top_right(), set);
	line_to(bbox.bottom_right(), set);
	line_to(bbox.bottom_left(), set);
	line_to(bbox.top_left(), set);
}

void __not_in_flash_func(XY2Galvo::draw_polyline) (uint count, std::function<Point()> next_point, const LaserSet& set, uint flags) {
	bool closed   = flags == POLYLINE_CLOSED;
	bool no_start = flags & POLYLINE_NO_START;
	bool no_end   = flags & POLYLINE_NO_END;

	Point start;
	if (!no_start && count > 0) {
        count--;
        start = next_point();
        move_to(start);
    }
	if (count == 0) return;

	uint laser_on_delay = no_start ? 0 : set.delay_a;
	uint delay = set.delay_m;

	while (count > 0) {
        count --;
		Point dest = next_point();
		if (count==0 && !no_end) delay = set.delay_e;
		draw_to(dest, set.speed, set.pattern, laser_on_delay, delay);
	}

	if (closed) draw_to(start, set.speed, set.pattern, laser_on_delay, set.delay_e);
}


// --- Public API methods (run on Core 0) ---

void XY2Galvo::moveTo (const Point& p) {
	laser_queue.push(CMD_MOVETO);
	laser_queue.push(p);
}
void XY2Galvo::drawTo (const Point& p, const LaserSet& set) {
	laser_queue.push(CMD_DRAWTO);
	laser_queue.push(&set);
	laser_queue.push(p);
}
void XY2Galvo::drawLine (const Point& p1, const Point& p2, const LaserSet& set) {
	laser_queue.push(CMD_LINE);
	laser_queue.push(&set);
	laser_queue.push(p1);
	laser_queue.push(p2);
}
void XY2Galvo::drawRect (const Rect& rect, const LaserSet& set) {
	laser_queue.push(CMD_RECT);
	laser_queue.push(&set);
	laser_queue.push(rect);
}
void XY2Galvo::drawPolyLine (uint count, const Point points[], const LaserSet& set, PolyLineOptions flags) {
	laser_queue.push(CMD_POLYLINE);
	laser_queue.push(&set);
	laser_queue.push(uint(flags));
	laser_queue.push(count);
	for (uint i=0; i<count; i++) laser_queue.push(points[i]);
}
void XY2Galvo::drawPolygon (uint count, const Point points[], const LaserSet& set) {
	drawPolyLine(count, points, set, POLYLINE_CLOSED);
}

void XY2Galvo::update_transformation() {
    static_assert(sizeof(Data32) == sizeof(FLOAT), "Size mismatch");
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
void XY2Galvo::rotate(FLOAT rad) {
    transformation0.rotate(rad);
    update_transformation();
}
void XY2Galvo::scale(FLOAT s) {
    transformation0.scale(s);
    update_transformation();
}
void XY2Galvo::scale(FLOAT sx, FLOAT sy) {
    transformation0.scale(sx, sy);
    update_transformation();
}
void XY2Galvo::addOffset(FLOAT dx, FLOAT dy) {
    transformation0.addOffset(dx, dy);
    update_transformation();
}

