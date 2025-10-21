#ifndef XY2Galvo_H
#define XY2Galvo_H

#include "Arduino.h"
#include <cmath>
#include <cstdint>
#include <functional>

#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"

#include "basic_geometry.h"
#include "Queue.h"
#include "XY2-100.pio.h"

// --- Pin Definitions ---
// The PIO program uses differential signaling for Clock, Sync, X, and Y.
// Each signal uses a pair of consecutive GPIO pins. You only need to define
// the base pin for each pair. The library and PIO will handle the second pin
// automatically (base_pin + 1).

constexpr uint32_t XY2_SM_CLOCK = 8000000;
constexpr uint32_t XY2_DATA_CLOCK = 100000;

// Differential Clock Signal
constexpr uint PIN_XY2_CLOCK      = 8;  // CLOCK+ goes here
constexpr uint PIN_XY2_CLOCK_NEG  = PIN_XY2_CLOCK + 1; // CLOCK- goes here (9)

// Differential Sync Signal (part of the clock/sync group)
constexpr uint PIN_XY2_SYNC       = PIN_XY2_CLOCK + 2; // SYNC+ goes here (10)
constexpr uint PIN_XY2_SYNC_NEG   = PIN_XY2_SYNC + 1;  // SYNC- goes here (11)

// Differential Y-Axis Data
constexpr uint PIN_XY2_Y          = 12; // Y+ goes here
constexpr uint PIN_XY2_Y_NEG      = PIN_XY2_Y + 1;     // Y- goes here (13)

// Differential X-Axis Data
constexpr uint PIN_XY2_X          = 14; // X+ goes here
constexpr uint PIN_XY2_X_NEG      = PIN_XY2_X + 1;     // X- goes here (15)

// Single-ended Signals
constexpr uint PIN_XY2_SYNC_XY    = 16; // Synchronization for PIO state machines
constexpr uint PIN_XY2_LASER	  = 22; // Laser ON/OFF control

// --- Scanner Hardware Constants ---
constexpr int32_t SCANNER_MAX = 0xffff;
constexpr float SCANNER_MAX_SWIVELS = 15000.0f / 120.0f;
constexpr float SCANNER_WIDTH = 0x10000;
constexpr float SCANNER_MAX_STEP_SPEED = SCANNER_MAX_SWIVELS * SCANNER_WIDTH / XY2_DATA_CLOCK;
constexpr float MAX_SPEED_UPS = SCANNER_MAX_STEP_SPEED * XY2_DATA_CLOCK;

// --- Motion Smoothing ---
// This determines how frequently the planner on Core 0 sends updates for slow movements.
// Higher values result in smoother motion at the cost of more commands in the queue.
constexpr float TARGET_UPDATE_RATE_HZ = 100.0f;
// --- End Pin Definitions & Constants ---

#define PIO_XY2 pio0

struct LaserSet
{
	float speed;    // Speed in galvo units per second.
	uint pattern;
	uint delay_a;	// steps to wait after start before laser ON (*10 us)
	uint delay_m;	// steps to wait at middle points in polygon lines (*10 us)
	uint delay_e;	// steps to wait with laser ON after end of line (*10 us)
};

enum DrawCmd
{
	CMD_END = 0,
	CMD_MOVETO,
	CMD_LINETO,
	CMD_DWELL,
	CMD_RESET_TRANSFORMATION,
	CMD_SET_TRANSFORMATION,
    CMD_SET_TRANSFORMATION_3D,
};

union Data32
{
	DrawCmd cmd;
	float f;
	uint  u;
	int   i;
	Data32(DrawCmd c) : cmd(c){}
	Data32(float f)   : f(f)  {}
	Data32(uint  u)   : u(u)  {}
	Data32(int   i)   : i(i)  {}
	Data32(){}
	~Data32(){}
};

enum PolyLineOptions
{
	POLYLINE_DEFAULT  = 0,
	POLYLINE_CLOSED   = 4
};

class LaserQueue : public Queue<Data32, 256>
{
public:
	void push (Data32 data);
	void push (const Point& p);
	void push (const Rect& r);
	Data32 pop ();
	Point pop_Point ();
	Rect pop_Rect ();
};

extern LaserQueue laser_queue;

class XY2Galvo
{
public:
	XY2Galvo();

	void init();
	void start();

    // Drawing commands
	void moveTo (const Point& dest);
	void drawLine (const Point& start, const Point& dest, const LaserSet&);
	void drawRect (const Rect& rect, const LaserSet&);
	void drawPolyLine (uint count, const Point points[], const LaserSet&, PolyLineOptions=POLYLINE_DEFAULT);
	void drawPolygon (uint count, const Point points[], const LaserSet&);

    // Timing command
    void wait(uint32_t microseconds);

    // Transformation commands
	void resetTransformation();
	void setTransformation (const Transformation& transformation);
    void pushTransformation();
    void popTransformation();
    void rotate(float rad);
    void scale(float s);
    void scale(float sx, float sy);
    void addOffset(float dx, float dy);


private:
	static void worker();

	void update_transformation ();
	static uint delayed_laser_value (uint value);

	static void pio_wait_free();
	static void pio_send_data (float x, float y, uint32_t laser);
	static void send_data_blocking (const Point& p, uint32_t laser);

	// Core 1 (worker) functions
	static void execute_lineto (Point dest, uint pattern);
	
    // PIO state machines
	static constexpr uint sm_laser  = 0;
	static constexpr uint sm_clock  = 1;
	static constexpr uint sm_x      = 2;
	static constexpr uint sm_y      = 3;

    // State variables
    static Point pos0;
    static Transformation transformation0;
    static Transformation transformation1;
    static Transformation transformation_stack[8];
    static uint transformation_stack_index;
};

#endif // XY2Galvo_H

