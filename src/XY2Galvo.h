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

// --- Merged from standard_types.h & cdefs.h ---
using FLOAT = float;
typedef const char* cstr;
//template<typename T> static inline T min(T a,T b) { return a<=b?a:b; }
//template<typename T> static inline T max(T a,T b) { return a>=b?a:b; }
// --- End Merged ---


// --- Merged from settings.h ---
constexpr uint32_t XY2_SM_CLOCK = 8000000;
constexpr uint32_t XY2_DATA_CLOCK = 100000;

// --- Pin Definitions ---
// The PIO program uses differential signaling for Clock, Sync, X, and Y.
// Each signal uses a pair of consecutive GPIO pins. You only need to define
// the base pin for each pair. The library and PIO will handle the second pin
// automatically (base_pin + 1).

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


constexpr int32_t SCANNER_MAX = 0xffff;
constexpr float SCANNER_MAX_SWIVELS = 15000.0f / 120.0f;
constexpr float SCANNER_WIDTH = 0x10000;
constexpr float SCANNER_MAX_SPEED = SCANNER_MAX_SWIVELS * SCANNER_WIDTH / XY2_DATA_CLOCK;
// --- End Merged ---

#define PIO_XY2 pio0

struct LaserSet
{
	FLOAT speed;
	uint pattern;
	uint delay_a;	// steps to wait after start before laser ON
	uint delay_m;	// steps to wait at middle points in polygon lines
	uint delay_e;	// steps to wait with laser ON after end of line
};

enum DrawCmd
{
	CMD_END = 0,
	CMD_MOVETO,
	CMD_DRAWTO,
	CMD_LINETO,
	CMD_LINE,
	CMD_RECT,
	CMD_POLYLINE,
	CMD_RESET_TRANSFORMATION,
	CMD_SET_TRANSFORMATION,
    CMD_SET_TRANSFORMATION_3D,
};

union Data32
{
	DrawCmd cmd;
	const LaserSet* set;
	FLOAT f;
	uint  u;
	int   i;
	Data32(DrawCmd c) : cmd(c){}
	Data32(FLOAT f)   : f(f)  {}
	Data32(uint  u)   : u(u)  {}
	Data32(int   i)   : i(i)  {}
	Data32(const LaserSet* s) : set(s) {}
	Data32(){}
	~Data32(){}
};

enum PolyLineOptions
{
	POLYLINE_DEFAULT  = 0,
	POLYLINE_NO_START = 1,
	POLYLINE_NO_END   = 2,
	POLYLINE_INFINITE = 3,
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
	void drawTo (const Point& dest, const LaserSet&);
	void drawLine (const Point& start, const Point& dest, const LaserSet&);
	void drawRect (const Rect& rect, const LaserSet&);
	void drawPolyLine (uint count, const Point points[], const LaserSet&, PolyLineOptions=POLYLINE_DEFAULT);
	void drawPolygon (uint count, const Point points[], const LaserSet&);

    // Transformation commands
	void resetTransformation();
	void setTransformation (const Transformation& transformation);
    void pushTransformation();
    void popTransformation();
    void rotate(FLOAT rad);
    void scale(FLOAT s);
    void scale(FLOAT sx, FLOAT sy);
    void addOffset(FLOAT dx, FLOAT dy);


private:
	static void worker();

	void update_transformation ();
	static uint delayed_laser_value (uint value);

	static void pio_wait_free();
	static void pio_send_data (FLOAT x, FLOAT y, uint32_t laser);
	static void send_data_blocking (const Point& p, uint32_t laser);

	static void draw_to (Point dest, FLOAT speed, uint laser_on_pattern, uint& laser_on_delay, uint end_delay);
	static void move_to (const Point& dest);
	static void line_to (const Point& dest, const LaserSet&);
	static void draw_line (const Point& start, const Point& dest, const LaserSet&);
	static void draw_rect (const Rect& rect, const LaserSet&);
	static void draw_polyline (uint count, std::function<Point()> next_point, const LaserSet&, uint flags);

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