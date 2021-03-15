#include <iostream>
#include "linedrawer.h"


/**/
int draw_x_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
	int dx = ex - sx;
	int dy = ey - sy;
	int dir = 1;
	if (sx > ex)
	{
		dir = -1;
	}
	int yi = 1;
	if (dy < 0)
	{
		yi = -1;
		dy = -dy;
	}
	int D = 2 * dy - dx;

	int x = sx;
	int y = sy;

	while (x != ex)
	{
		fb->plotPixel(x, y, 1.0f, 1.0f, 1.0f);
		if (D > 0)
		{
			y += yi;
			D = D - 2 * dx;
		}
		D = D + 2 * dy;
		x += dir;
	}
}

int draw_y_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
	int dx = ex - sx;
	int dy = ey - sy;
	int dir = 1;
	if (sy > ey)
	{
		dir = -1;
	}
	int xi = 1;
	if (dx < 0)
	{
		xi = -1;
		dx = -dx;
	}
	int D = 2 * dx - dy;

	int x = sx;
	int y = sy;

	while (y != ey)
	{
		fb->plotPixel(x, y, 1.0f, 1.0f, 1.0f);
		if (D > 0)
		{
			x += xi;
			D = D - 2 * dy;
		}
		D = D + 2 * dx;
		y += dir;
	}
}

int draw_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
	if ((sx == ex) && (sy == ey))
	{
		return fb->plotPixel(sx, sy, 1.0f, 1.0f, 1.0f);

	}
	else if (((ex - sx)* (ex - sx)) >= ((ey - sy)* (ey - sy)))
	{
		if (sx > ex)
		{
			return draw_x_line(fb, ex, ey, sx, sy);
		}
		else
		{
			return draw_x_line(fb, sx, sy, ex, ey);
		}

	}
	else
	{
		if (sy > ey)
		{
			return draw_y_line(fb, ex, ey, sx, sy);
		}
		else
		{
			return draw_y_line(fb, sx, sy, ex, ey);
		}
	}
}

/*
int draw_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
  if ((sx == ex) && (sy==ey))
  {
    return fb->plotPixel(sx, sy, 1.0f, 1.0f, 1.0f);
    
  } else if (((ex-sx)* (ex-sx)) >= ((ey-sy)* (ey-sy)))
  {
    return draw_x_line(fb, sx, sy, ex, ey);
    
  } else
  {
    return draw_y_line(fb, sx, sy, ex, ey);
  }
}

int draw_x_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
	int dir = 1;
	if (sx > ex)
	{
		dir = -1;
	}

	int   x = sx;
	float y = (float)sy;
	float slope = ((float)ey - (float)sy) / ((float)ex - (float)sx);
	slope = slope * dir;

	while (x != ex)
	{
		fb->plotPixel(x, (int)y, 1.0f, 1.0f, 1.0f);

		y += slope;

		x += dir;
	}

}

int draw_y_line(FrameBuffer *fb, int sx, int sy, int ex, int ey)
{
	int dir = 1;
	if (sy > ey)
	{
		dir = -1;
	}

	int   y = sy;
	float x = (float)sx;
	float slope = ((float)ex - (float)sx) / ((float)ey - (float)sy);
	slope = slope * dir;

	while (y != ey)
	{
		fb->plotPixel((int)x, y, 1.0f, 1.0f, 1.0f);

		x += slope;

		y += dir;
	}

}
*/