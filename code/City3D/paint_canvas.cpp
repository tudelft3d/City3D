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

#include "paint_canvas.h"
#include "main_window.h"
#include "dlg/wgt_render.h"
#include "../3rd_party/3rd_QGLViewer-2.6.3/manipulatedCameraFrame.h"
#include "../3rd_party/3rd_QGLViewer-2.6.3/camera.h"
#include "../basic/file_utils.h"
#include "../basic/stop_watch.h"
#include "../model/map.h"
#include "../model/map_editor.h"
#include "../model/map_geometry.h"
#include "../model/point_set.h"
#include "../model/vertex_group.h"
#include "../renderer/opengl_info.h"
#include "../renderer/mesh_render.h"
#include "../renderer/point_set_render.h"
#include "../renderer/rendering_styles.h"
#include "../renderer/tessellator.h"
#include "../model/map_circulators.h"
#include "../method/method_global.h"
#include "../method/reconstruction.h"
#include "../method/point_set_normals.h"

#include <QFileDialog>
#include <QMouseEvent>
#include <QMessageBox>
#include <QColorDialog>

#include <cassert>
#include <fstream>
#include <algorithm>


using namespace qglviewer;
PaintCanvas::PaintCanvas(QWidget *parent, QGLFormat format)
	: QGLViewer(format, parent)
	, coord_system_region_size_(150)
	, show_coord_sys_(true)
	, point_set_(nil)
	, foot_print_(nil)
	, reconstruction_(nil)
	, show_point_set_(true)
	, show_foot_print_(true)
	, show_reconstruction_(true)
{
	setFPSIsDisplayed(true);

	main_window_ = dynamic_cast<MainWindow*>(parent);

	light_pos_ = vec3(0.27f, 0.27f, 0.92f);

	//////////////////////////////////////////////////////////////////////////

	// Move camera according to viewer type (on X, Y or Z axis)
	camera()->setPosition(qglviewer::Vec(1.0, 1.0, 1.0));

	camera()->lookAt(sceneCenter());
	camera()->setType(qglviewer::Camera::PERSPECTIVE);
	camera()->showEntireScene();

	point_set_render_ = new PointSetRender(this);
}


PaintCanvas::~PaintCanvas() {
//	// this is required by the following destruction of textures, shaders, etc.
//	makeCurrent();

	delete point_set_render_;
	Tessellator::terminate();
	MeshRender::terminate();

	clear();
}


void PaintCanvas::clear() {
	if (point_set_)
		point_set_.forget();

	if (foot_print_)
		foot_print_.forget();

	if (reconstruction_)
		reconstruction_.forget();
}


void PaintCanvas::init()
{
	GLenum err = glewInit();
	if (GLEW_OK != err) {
		// Problem: glewInit failed, something is seriously wrong. 
		Logger::err("-") << glewGetErrorString(err) << std::endl;
	}

	//////////////////////////////////////////////////////////////////////////

	setStateFileName("");

	// Default value is 0.005, which is appropriate for most applications. 
	// A lower value will prevent clipping of very close objects at the 
	// expense of a worst Z precision.
	camera()->setZNearCoefficient(0.005f);

	// Default value is square root of 3.0 (so that a cube of size 
	// sceneRadius() is not clipped).
	camera()->setZClippingCoefficient(std::sqrt(3.0f));

	camera()->setViewDirection(qglviewer::Vec(0.0, 1.0, 0.0));
	camera()->setType(qglviewer::Camera::PERSPECTIVE);
	showEntireScene();

	camera()->frame()->setSpinningSensitivity(/*1.0f*/100.0f);
	setMouseTracking(true);

	//////////////////////////////////////////////////////////////////////////
	// I like the inverse direction
	setShortcut(MOVE_CAMERA_LEFT, Qt::Key_Right);
	setShortcut(MOVE_CAMERA_RIGHT, Qt::Key_Left);
	setShortcut(MOVE_CAMERA_UP, Qt::Key_Down);
	setShortcut(MOVE_CAMERA_DOWN, Qt::Key_Up);

	setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, CAMERA, SCREEN_ROTATE);
	setMouseBinding(Qt::ControlModifier, Qt::LeftButton, CAMERA, ZOOM_ON_REGION);

	//////////////////////////////////////////////////////////////////////////

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  //GL_ONE_MINUS_DST_ALPHA

	// for transparent 
	glAlphaFunc(GL_GREATER, 0);
	glEnable(GL_ALPHA_TEST);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	//////////////////////////////////////////////////////////////////////////

	QColor bkgrd_color = Qt::white;
	setBackgroundColor(bkgrd_color);

	//////////////////////////////////////////////////////////////////////////

	//float pos[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	float pos[] = { float(light_pos_.x), float(light_pos_.y), float(light_pos_.z), 0.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, pos);

	// Setup light parameters
	// Liangliang: my experience is that it hurts a lot the rendering performance.
	// If not needed, you can set to "GL_FALSE" (e.g., for large scale scenes).
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE); /*GL_FALSE*/

	// how specular reflection angles are computed. GL_TRUE will introduce artifact for glu tess with specular
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);

	// Makes specular lighting work in texture mapping mode.
	glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);

	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);

	//////////////////////////////////////////////////////////////////////////

	// specify the specular and shininess
	float specular[] = { 0.6f, 0.6f, 0.6f, 0.5f };
	float shininess = 64.0f;
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	////////////////////////////////////////////////////////////////////////////

	// 	// make the back side different
	// 	float back_specular[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	// 	float back_shininess = 128;
	// 	float ambient_back[]  = {0.0f, 1.0f, 0.0f, 1.0f};
	// 	glMaterialfv(GL_BACK, GL_SPECULAR, back_specular);
	// 	glMaterialf(GL_BACK, GL_SHININESS, back_shininess);
	// 	glMaterialfv(GL_BACK, GL_AMBIENT, ambient_back);

	////////////////////////////////////////////////////////////////////////

	// to use facet color, the GL_COLOR_MATERIAL should be enabled
	glEnable(GL_COLOR_MATERIAL);
	// to use material color, the GL_COLOR_MATERIAL should be disabled
	//glDisable(GL_COLOR_MATERIAL);

	setFPSIsDisplayed(false);
}


void PaintCanvas::draw() {
	if (show_coord_sys_)
		drawCornerAxis();

	if (point_set_ && show_point_set_ && point_set_render_)
		point_set_render_->draw(point_set_);

	if (reconstruction_ || foot_print_) {
		glEnable(GL_LIGHTING);
		Tessellator::draw();

		glDisable(GL_LIGHTING);
		MeshRender::draw();
	}

	//const static QFont font("Tekton Pro Ext", 12/*, QFont::Bold*/); // "Times", "Helvetica", "Bradley Hand ITC"
	//if (show_hint_text_) {
	//	if (!hint_text_.isEmpty()) {
	//		glColor3f(0, 0, 0.7f);
	//		drawText(30, 40, hint_text_, font);
	//		if (!hint_text2nd_.isEmpty())
	//			drawText(30, 70, hint_text2nd_, font);
	//	}
	//}

	//if (show_mouse_hint_) {
	//	glColor3f(0, 0, 0);
	//	drawText(30, 120, "Mouse Operations:", font);
	//	drawText(30, 150, "  - Orbit: left button", font);
	//	drawText(30, 180, "  - Pan:   right button", font);
	//	drawText(30, 210, "  - Zoom:  wheel", font);
	//}
}

void PaintCanvas::mouseMoveEvent(QMouseEvent* e)
{
	bool found = false;
	qglviewer::Vec p = camera()->pointUnderPixel(e->pos(), found);
	main_window_->showCoordinateUnderMouse(vec3(p.x, p.y, p.z), found);

	QGLViewer::mouseMoveEvent(e);
}



void PaintCanvas::keyPressEvent(QKeyEvent *e) {
	e->ignore();
}


void PaintCanvas::snapshotScreen() {
	bool need_hide = show_coord_sys_;
	if (need_hide)
		show_coord_sys_ = false;  // hide the coord system temporally

// 	show_hint_text_ = false;
// 	show_mouse_hint_ = false;
	saveSnapshot(false);
// 	show_hint_text_ = true;
// 	show_mouse_hint_ = true;

	if (need_hide)
		show_coord_sys_ = true;

	updateGL();
}


void PaintCanvas::update_graphics() {
	updateGL();

	// This approach has significant drawbacks. For example, imagine you wanted to perform two such loops 
	// in parallel-calling one of them would effectively halt the other until the first one is finished 
	// (so you can't distribute computing power among different tasks). It also makes the application react
	// with delays to events. Furthermore the code is difficult to read and analyze, therefore this solution
	// is only suited for short and simple problems that are to be processed in a single thread, such as 
	// splash screens and the monitoring of short operations.
	QCoreApplication::processEvents();
}

void PaintCanvas::update_all() {
	updateGL();
	main_window_->updateStatusBar();

	// This approach has significant drawbacks. For example, imagine you wanted to perform two such loops 
	// in parallel-calling one of them would effectively halt the other until the first one is finished 
	// (so you can't distribute computing power among different tasks). It also makes the application react
	// with delays to events. Furthermore the code is difficult to read and analyze, therefore this solution
	// is only suited for short and simple problems that are to be processed in a single thread, such as 
	// splash screens and the monitoring of short operations.
	QCoreApplication::processEvents();
}


void PaintCanvas::showCoordinateSystem(bool b) {
	show_coord_sys_ = b;
	updateGL();
}


void PaintCanvas::fitScreen(const Box3d& box) {
	qglviewer::Vec vmin(box.x_min(), box.y_min(), box.z_min());
	qglviewer::Vec vmax(box.x_max(), box.y_max(), box.z_max());

	setSceneBoundingBox(vmin, vmax);
	showEntireScene();
	updateGL();
}

void PaintCanvas::fitAll() {
	Box3d box;
	if (dynamic_cast<PointSet*>(pointSet()))
		box.add_box(dynamic_cast<PointSet*>(pointSet())->bbox());

	if (dynamic_cast<Map*>(footPrint()))
		box.add_box(Geom::bounding_box(dynamic_cast<Map*>(footPrint())));
	
	if (dynamic_cast<Map*>(reconstruction()))
		box.add_box(Geom::bounding_box(dynamic_cast<Map*>(reconstruction())));

	fitScreen(box);
}


void PaintCanvas::drawCornerAxis()
{
	glEnable(GL_MULTISAMPLE);

	// The viewport and the scissor are changed to fit the lower left
	// corner. Original values are saved.
	int viewport[4];
	int scissor[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetIntegerv(GL_SCISSOR_BOX, scissor);

	//////////////////////////////////////////////////////////////////////////

	// Axis viewport size, in pixels
	glViewport(0, 0, coord_system_region_size_, coord_system_region_size_);
	glScissor(0, 0, coord_system_region_size_, coord_system_region_size_);

	// The Z-buffer is cleared to make the axis appear over the
	// original image.
	glClear(GL_DEPTH_BUFFER_BIT);

	// Tune for best line rendering
	glLineWidth(3.0);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1, 1, -1, 1, -1, 1);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixd(camera()->orientation().inverse().matrix());

	float axis_size = 0.9f; // other 0.2 space for drawing the x, y, z labels
	drawAxis(axis_size);

	// Draw text id
	glColor3f(0, 0, 0);

	// Liangliang: It seems the renderText() func disables multi-sample.
	// Is this a bug in Qt ?
	GLboolean anti_alias = glIsEnabled(GL_MULTISAMPLE);
	const_cast<PaintCanvas*>(this)->renderText(axis_size, 0, 0, "X");
	const_cast<PaintCanvas*>(this)->renderText(0, axis_size, 0, "Y");
	const_cast<PaintCanvas*>(this)->renderText(0, 0, axis_size, "Z");
	if (anti_alias)
		glEnable(GL_MULTISAMPLE);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	//////////////////////////////////////////////////////////////////////////

	// The viewport and the scissor are restored.
	glScissor(scissor[0], scissor[1], scissor[2], scissor[3]);
	glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}


vec2 PaintCanvas::projectionOf(const vec3& p) {    // point to screen
	qglviewer::Vec v = camera()->projectedCoordinatesOf(qglviewer::Vec(p.x, p.y, p.z));
	return vec2(v.x, v.y);
}

vec3 PaintCanvas::unProjectionOf(double winx, double winy, double winz) {  // screen to point	
	qglviewer::Vec v = camera()->unprojectedCoordinatesOf(qglviewer::Vec(winx, winy, winz));
	return vec3(v.x, v.y, v.z);
}


void PaintCanvas::setFootPrint(Map* mesh) {
	bool scene_changed = false;
	if (foot_print_ || mesh == nil) {
		Tessellator::remove_mesh(foot_print_);
		MeshRender::remove_mesh(foot_print_);
		foot_print_.forget();
		scene_changed = true;
	}

	if (mesh) {
		foot_print_ = mesh;

		MapFacetAttribute<Color> color(foot_print_, "color");
		FOR_EACH_FACET(Map, foot_print_, it)
			color[it] = random_color();

		Tessellator::add_mesh(foot_print_);
		MeshRender::add_mesh(foot_print_);
		scene_changed = true;
	}

	if (scene_changed) {
		Tessellator::invalidate();
		MeshRender::invalidate();
	}
}


void PaintCanvas::setReconstruction(Map* mesh) {
	bool scene_changed = false;
	if (reconstruction_ || mesh == nil) {
		Tessellator::remove_mesh(reconstruction_);
		MeshRender::remove_mesh(reconstruction_);
		reconstruction_.forget();
		scene_changed = true;
	}

	if (mesh) {
		reconstruction_ = mesh;
		Tessellator::add_mesh(reconstruction_);
		MeshRender::add_mesh(reconstruction_);
		scene_changed = true;
	}

	if (scene_changed) {
		Tessellator::invalidate();
		MeshRender::invalidate();
	}
}


void PaintCanvas::setPointSet(PointSet* pset) { 
	if (point_set_)
		point_set_.forget();

	if (pset) {
		point_set_ = pset;
		if (foot_print_) {
			Tessellator::remove_mesh(foot_print_);
			MeshRender::remove_mesh(foot_print_);
			foot_print_.forget();
			Tessellator::invalidate();
			MeshRender::invalidate();
		}
		if (reconstruction_) {
			Tessellator::remove_mesh(reconstruction_);
			MeshRender::remove_mesh(reconstruction_);
			reconstruction_.forget();
			Tessellator::invalidate();
			MeshRender::invalidate();
		}
	}
}


Map* PaintCanvas::footPrint() const {
	return foot_print_;
}

Map* PaintCanvas::reconstruction() const {
	return reconstruction_;
}

PointSet* PaintCanvas::pointSet() const {
	return point_set_;
}


void PaintCanvas::setShowPointSet(bool b) {
	show_point_set_ = b;
	update_all();
}

void PaintCanvas::setProjectionMode(bool b) {
	if (b)
		camera()->setType(Camera::PERSPECTIVE);
	else
		camera()->setType(Camera::ORTHOGRAPHIC);

	update_graphics();
}

void PaintCanvas::setShowFootPrint(bool b) {
	if (b) {
		Tessellator::add_mesh(foot_print_);
		MeshRender::add_mesh(foot_print_);
		show_foot_print_ = true;
	}
	else {
		Tessellator::remove_mesh(foot_print_);
		MeshRender::remove_mesh(foot_print_);
		show_foot_print_ = false;
	}

	Tessellator::invalidate();
	MeshRender::invalidate();

	update_all();
}


void PaintCanvas::setShowReconstruction(bool b) {
	if (b) {
		Tessellator::add_mesh(reconstruction_);
		MeshRender::add_mesh(reconstruction_);
		show_reconstruction_ = true;
	}
	else {
		Tessellator::remove_mesh(reconstruction_);
		MeshRender::remove_mesh(reconstruction_);
		show_reconstruction_ = false;
	}

	Tessellator::invalidate();
	MeshRender::invalidate();

	update_all();
}

void PaintCanvas::saveStateAsMappleFormat() {
	std::string str = FileUtils::replace_extension(stateFileName().toStdString(), "state");
	QString name = QString::fromStdString(str);
	if (name.isEmpty())
		return;

	// Write the state to file
	std::ofstream output(name.toStdString().c_str());
	if (output.fail()) {
		QMessageBox::warning(window(), tr("Save state to file error"), tr("Unable to create file %1").arg(name));
		return;
	}

	//-----------------------------------------------------

	// first line is just a comment
	output << "<Mapple state file version 263>" << std::endl << std::endl;

	//-----------------------------------------------------

	// write foreground and background colors
	output << "<color>" << std::endl;
	QColor fc = foregroundColor();
	output << "\t foreground: " << fc.red() << " " << fc.green() << " " << fc.blue() << std::endl;
	QColor bc = backgroundColor();
	output << "\t background: " << bc.red() << " " << bc.green() << " " << bc.blue() << std::endl;
	output << "</color>" << std::endl << std::endl;

	//-----------------------------------------------------

	// Revolve or fly camera mode is not saved
	// ...

	//-----------------------------------------------------

	output << "<display>" << std::endl;
	output << "\t cameraIsEdited: " << cameraIsEdited() << std::endl;
	output << "\t gridIsDrawn: " << gridIsDrawn() << std::endl;
	output << "\t axisIsDrawn: " << axisIsDrawn() << std::endl;
	output << "\t FPSIsDisplayed: " << FPSIsDisplayed() << std::endl;
	output << "</display>" << std::endl << std::endl;

	//-----------------------------------------------------

	output << "<windowState>" << std::endl;
	output << "\t state: " << window()->windowState() << std::endl;;
	if (window()->windowState() == Qt::WindowNoState) {
		output << "\t size: " << window()->width() << " " << window()->height() << std::endl;
		output << "\t position: " << window()->pos().x() << " " << window()->pos().y() << std::endl;
	}
	output << "</windowState>" << std::endl << std::endl;

	//-----------------------------------------------------

	output << "<camera>" << std::endl;
	// Restore original QCamera zClippingCoefficient before saving.
	if (cameraIsEdited())
		camera()->setZClippingCoefficient(previousCameraZClippingCoefficient_);

	switch (camera()->type()) {
	case Camera::PERSPECTIVE:	output << "\t type: " << "PERSPECTIVE" << std::endl;	break;
	case Camera::ORTHOGRAPHIC:	output << "\t type: " << "ORTHOGRAPHIC" << std::endl;	break;
	}
	output << "\t zClippingCoefficient: " << QString::number(camera()->zClippingCoefficient()).toStdString() << std::endl;
	output << "\t zNearCoefficient: " << QString::number(camera()->zNearCoefficient()).toStdString() << std::endl;
	output << "\t sceneRadius: " << QString::number(camera()->sceneRadius()).toStdString() << std::endl;
	output << "\t orthoCoefficient: " << QString::number(camera()->orthoCoefficient()).toStdString() << std::endl;
	output << "\t fieldOfView: " << QString::number(camera()->fieldOfView()).toStdString() << std::endl;
	output << "\t sceneCenter: " << camera()->sceneCenter() << std::endl;

	// ManipulatedCameraFrame
	output << "\t position: " << camera()->frame()->position() << std::endl;
	output << "\t orientation: " << camera()->frame()->orientation() << std::endl;
	output << "\t wheelSens: " << camera()->frame()->wheelSensitivity() << std::endl;
	output << "\t rotSens: " << camera()->frame()->rotationSensitivity() << std::endl;
	output << "\t zoomSens: " << camera()->frame()->zoomSensitivity() << std::endl;
	output << "\t spinSens: " << camera()->frame()->spinningSensitivity() << std::endl;
	output << "\t transSens: " << camera()->frame()->translationSensitivity() << std::endl;

	output << "\t zoomsOnPivotPoint: " << camera()->frame()->zoomsOnPivotPoint() << std::endl;
	output << "\t pivotPoint: " << camera()->frame()->pivotPoint() << std::endl;
	output << "\t rotatesAroundUpVector: " << camera()->frame()->rotatesAroundUpVector() << std::endl;
	output << "\t flySpeed: " << camera()->frame()->flySpeed() << std::endl;
	output << "\t sceneUpVector: " << camera()->frame()->sceneUpVector() << std::endl;

	if (cameraIsEdited())
		// #CONNECTION# 5.0 from setCameraIsEdited()
		camera()->setZClippingCoefficient(5.0);
	output << "</camera>" << std::endl << std::endl;
}

void PaintCanvas::estimateNormals() {
	if (!point_set_) {
		Logger::warn("-") << "point cloud data does not exist" << std::endl;
		return;
	}
	PointSetNormals::estimate(point_set_);

	update_all();
}


void PaintCanvas::segmentation() {
    main_window_->updateWeights();
	Reconstruction recon;
    if (!foot_print_) {
		if (main_window_->want_footprint()) {
            auto foot_print = recon.generate_footprint(point_set_);
            setFootPrint(foot_print);
        }
		else
			return;
    }

	if (foot_print_) {
		recon.segmentation(point_set_, foot_print_);
		main_window_->wgtRender_->checkBoxPointSet->setChecked(false);

        // foot_print may have been simplified in segmentation, so update the viewer
        Tessellator::remove_mesh(foot_print_);
        MeshRender::remove_mesh(foot_print_);
        MapFacetAttribute<Color> color(foot_print_, "color");
        FOR_EACH_FACET(Map, foot_print_, it)
            color[it] = random_color();
        Tessellator::add_mesh(foot_print_);
        MeshRender::add_mesh(foot_print_);
        Tessellator::invalidate();
        MeshRender::invalidate();
    }
	else
		main_window_->wgtRender_->checkBoxPointSet->setChecked(true);

	main_window_->wgtRender_->checkBoxSegments->setChecked(true);
	main_window_->checkBoxShowReconstruction->setChecked(false);

	update_all();
}


void PaintCanvas::extractRoofs() {
    main_window_->updateWeights();
	Reconstruction recon;
	auto status = recon.extract_roofs(point_set_, foot_print_);
	main_window_->wgtRender_->checkBoxPointSet->setChecked(!status);
	main_window_->wgtRender_->checkBoxSegments->setChecked(true);
	main_window_->checkBoxShowReconstruction->setChecked(false);
	update_all();
}


void PaintCanvas::reconstruct() {
	if (!foot_print_) {
		Logger::warn("-") << "footprint does not exist. You must either load it or generate it by clicking the 'Segmentation' button" << std::endl;
		return;
	}

	setReconstruction(new Map);
	main_window_->updateWeights();
	Reconstruction recon;

	bool status = recon.reconstruct(point_set_, foot_print_, reconstruction_, main_window_->active_solver(), show_reconstruction_);
	if (!status)
		setReconstruction(nil);

	main_window_->checkBoxShowReconstruction->setChecked(true);
	main_window_->wgtRender_->checkBoxPointSet->setChecked(false);
	main_window_->wgtRender_->checkBoxSegments->setChecked(!status);
	main_window_->wgtRender_->checkBoxSurface->setChecked(true);
	main_window_->wgtRender_->checkBoxPerFaceColor->setChecked(false);
	main_window_->wgtRender_->checkBoxSharpEdges->setChecked(true);

	update_all();
}
