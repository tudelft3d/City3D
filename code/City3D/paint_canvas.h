/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef PAINTCANVAS_H
#define PAINTCANVAS_H

#include "../3rd_party/3rd_glew/include/GL/glew.h"
#include "../3rd_party/3rd_QGLViewer-2.6.3/qglviewer.h"
#include "../basic/color.h"
#include "../math/math_types.h"
#include "../basic/canvas.h"
#include "../model/point_set.h"
#include "../model/map.h"


class MainWindow;
class PointSetRender;

class PaintCanvas : public QGLViewer
{
	Q_OBJECT

public:
	PaintCanvas(QWidget *parent, QGLFormat format);
	~PaintCanvas();

public:

	void update_graphics();
	void update_all();

	//////////////////////////////////////////////////////////////////////////

	MainWindow* mainWindow() const { return main_window_; }

	vec2 projectionOf(const vec3& p);          // point to screen
	vec3 unProjectionOf(double winx, double winy, double winz);  // screen to point

	//////////////////////////////////////////////////////////////////////////
	
	void setPointSet(PointSet* pset);
	void setFootPrint(Map* m);
	void setReconstruction(Map* m);

	// the active object
	Map*		footPrint() const ;
	Map*		reconstruction() const;
	PointSet*	pointSet() const;
	
	void fitScreen(const Box3d& box);

	PointSetRender* point_set_render() const { return point_set_render_; }

	void clear();

	//////////////////////////////////////////////////////////////////////////

protected:
	virtual void draw();
	virtual void init();

	// Mouse events functions
	virtual void mouseMoveEvent(QMouseEvent *e);

	// Keyboard events functions
	virtual void keyPressEvent(QKeyEvent *e);

public Q_SLOTS:
	void setProjectionMode(bool);
	void fitAll() ;
	void snapshotScreen();
	void showCoordinateSystem(bool);

	//////////////////////////////////////////////////////////////////////////

	void estimateNormals();

	void segmentation();
	void extractRoofs();
	void reconstruct();

	void setShowPointSet(bool);
	void setShowFootPrint(bool);
	void setShowReconstruction(bool);

	void saveStateAsMappleFormat();

private :
	void drawCornerAxis();

protected:
	MainWindow*	main_window_;
	vec3		light_pos_;

	int			coord_system_region_size_;
	bool		show_coord_sys_;

	Map::Ptr		footprint_;
	Map::Ptr		reconstruction_;
	PointSet::Ptr	point_set_;

	bool			show_point_set_;
	PointSetRender* point_set_render_;
	
	bool show_footprint_;
	bool show_reconstruction_;
};


#endif // PAINTCANVAS_H
