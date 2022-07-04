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

#include <QMessageBox>
#include <QFileDialog>
#include <QLabel>
#include <QStatusBar>
#include <QSettings>
#include <QCloseEvent>
#include <QPlainTextEdit>
#include <QGroupBox>
#include <QColorDialog>
#include <QProgressBar>
#include <QMimeData>
#include <QComboBox>
#include <QMenu>

#include "main_window.h"
#include "paint_canvas.h"

#include "dlg/wgt_render.h"
#include "dlg/weight_panel_click.h"
#include "dlg/weight_panel_manual.h"

#include "../basic/logger.h"
#include "../basic/file_utils.h"
#include "../model/map.h"
#include "../model/point_set.h"
#include "../model/map_attributes.h"
#include "../model/map_copier.h"
#include "../model/map_builder.h"
#include "../model/map_io.h"
#include "../model/map_serializer_json.h"
#include "../model/map_enumerator.h"
#include "../model/point_set_io.h"
#include "../method/method_global.h"
#include "../basic/attribute_serializer.h"


MainWindow::MainWindow(QWidget *parent, Qt::WindowFlags flags)
: QMainWindow(parent, flags)
, curDataDirectory_(".")
{	
	setupUi(this);

	//////////////////////////////////////////////////////////////////////////

	Logger::initialize();
	Logger::instance()->register_client(this);
	Logger::instance()->set_value(Logger::LOG_REGISTER_FEATURES, "*"); // log everything
	Logger::instance()->set_value(Logger::LOG_FILE_NAME, "City3D.log");
	//Logger::instance()->set_value("log_features",
	//	"EigenSolver;MapBuilder;MapParameterizer\
	//	LinearSolver");

	// Liangliang: added the time stamp in the log file
	std::string tstr = String::from_current_time();
	Logger::out("") << "--- started at: " << tstr << " ---" << std::endl;
	
	Progress::instance()->set_client(this) ;

	AttributeSerializer::initialize();

	register_attribute_type<int>("int");
	register_attribute_type<float>("float");
	register_attribute_type<double>("double");
	register_attribute_type<bool>("bool") ;
	register_attribute_type<std::string>("string") ;
	register_attribute_type<vec2>("vec2") ;
	register_attribute_type<vec3>("vec3") ;
	register_attribute_type<vec4>("vec4") ;
	register_attribute_type<mat2>("mat2") ;
	register_attribute_type<mat3>("mat3") ;
	register_attribute_type<mat4>("mat4") ;
	register_attribute_type<Color>("Color");

	// ensure backward compatibility with .eobj files generated before.
	// PointXd/VectorXd do not exist anymore.
	register_attribute_type_alias("Vector2d", "vec2") ;
	register_attribute_type_alias("Vector3d", "vec3") ;
	register_attribute_type_alias("Point2d", "vec2") ;
	register_attribute_type_alias("Point3d", "vec3") ;

	//////////////////////////////////////////////////////////////////////////

	// Setup the format to allow anti-aliasing if the graphic driver allows this.
	QGLFormat format = QGLFormat::defaultFormat();
	format.setProfile(QGLFormat::CompatibilityProfile);
	format.setSampleBuffers(true); // you can also call setOption(QGL::SampleBuffers)
	format.setSamples(8);  // 8 is enough

	mainCanvas_ = new PaintCanvas(this, format);
	mainCanvas_->setAttribute(Qt::WA_MouseTracking);
	mainCanvas_->setMouseTracking(true);
	layoutCanvas->addWidget(mainCanvas_);

	//////////////////////////////////////////////////////////////////////////
	 createRenderingPanel();

	createActions();
	createStatusBar();
	createToolBar();

	readSettings();
	setWindowTitle("City3D");

	setContextMenuPolicy(Qt::CustomContextMenu);
	setWindowIcon(QIcon(":/Resources/City3D.png"));

	setAcceptDrops(true);

	setWindowState(Qt::WindowMaximized);
	setFocusPolicy(Qt::StrongFocus);
	showMaximized();
}


MainWindow::~MainWindow()
{
	if (wgtRender_)	delete wgtRender_;

	//////////////////////////////////////////////////////////////////////////

	AttributeSerializer::terminate();
	Progress::instance()->set_client(nil) ;
	Logger::instance()->unregister_client(this);
	Logger::terminate();
}


void MainWindow::out_message(const std::string& msg) {
	plainTextEditOutput->moveCursor(QTextCursor::End);
	plainTextEditOutput->insertPlainText(QString::fromStdString(msg));
	plainTextEditOutput->repaint();
	plainTextEditOutput->update();
}


void MainWindow::warn_message(const std::string& msg) {
	plainTextEditOutput->moveCursor(QTextCursor::End);
	plainTextEditOutput->insertPlainText(QString::fromStdString(msg));
	plainTextEditOutput->repaint();
	plainTextEditOutput->update();
}


void MainWindow::err_message(const std::string& msg) {
	plainTextEditOutput->moveCursor(QTextCursor::End);
	plainTextEditOutput->insertPlainText(QString::fromStdString(msg));
	plainTextEditOutput->repaint();
	plainTextEditOutput->update();
}


void MainWindow::status_message(const std::string& msg, int timeout) {
	statusBar()->showMessage(QString::fromStdString(msg), timeout);
}


void MainWindow::notify_progress(std::size_t value) {
	progress_bar_->setValue(value);
	progress_bar_->setTextVisible(value != 0);
	mainCanvas_->update_all();
}


void MainWindow::dragEnterEvent(QDragEnterEvent *e) {
	if (e->mimeData()->hasUrls()) {
		e->acceptProposedAction();
	}
}

void MainWindow::dropEvent(QDropEvent *e) {
    foreach (const QUrl &url, e->mimeData()->urls()) {
        const QString &fileName = url.toLocalFile();
        doOpen(fileName);
    }
}


void MainWindow::createActions() {
	connect(actionOpen, SIGNAL(triggered()), this, SLOT(open()));
	
	connect(actionProjectionMode, SIGNAL(toggled(bool)), mainCanvas_, SLOT(setProjectionMode(bool)));
	mainCanvas_->setProjectionMode(true);

	connect(actionSnapshot, SIGNAL(triggered()), mainCanvas_, SLOT(snapshotScreen()));

	connect(checkBoxShowFootPrint, SIGNAL(toggled(bool)), mainCanvas_, SLOT(setShowFootPrint(bool)));
	connect(checkBoxShowReconstruction, SIGNAL(toggled(bool)), mainCanvas_, SLOT(setShowReconstruction(bool)));

	connect(actionSegmentation, SIGNAL(triggered()), mainCanvas_, SLOT(segmentation()));
	connect(actionExtractRoofs, SIGNAL(triggered()), mainCanvas_, SLOT(extractRoofs()));
	connect(actionReconstruction, SIGNAL(triggered()), mainCanvas_, SLOT(reconstruct()));
	
	wgtRender_ = new WgtRender(this);
	layoutRenderer->addWidget(wgtRender_);

	// about menu
	connect(actionAbout, SIGNAL(triggered()), this, SLOT(about()));
}

void MainWindow::createRenderingPanel() {
	default_fitting_ = truncate_digits(Method::lambda_data_fitting, 3);
    default_height_ = truncate_digits(Method::lambda_model_height, 3);
	default_complexity_ = truncate_digits(Method::lambda_model_complexity, 3);

	panelClick_ = new WeightPanelClick(this);
	panelManual_ = new WeightPanelManual(this);

	verticalLayoutWeights->addWidget(panelClick_);
	verticalLayoutWeights->addWidget(panelManual_);
	panelManual_->setVisible(false);

	connect(panelClick_, SIGNAL(weights_changed()), panelManual_, SLOT(updateUI()));

	connect(pushButtonDefaultWeight, SIGNAL(pressed()), this, SLOT(resetWeights()));
	connect(checkBoxManualInputWeights, SIGNAL(toggled(bool)), this, SLOT(setManualInputWeights(bool)));
}


void MainWindow::updateWeights() {
	if (panelManual_->isEnabled()) {
		panelManual_->updateWeights();
		panelClick_->updateUI();
	}
}

void MainWindow::resetWeights() {
	Method::lambda_data_fitting = default_fitting_;
	Method::lambda_model_height = default_height_;
	Method::lambda_model_complexity = default_complexity_;

	panelClick_->updateUI();
	panelManual_->updateUI();
}


void MainWindow::closeEvent(QCloseEvent *event)
{
	writeSettings();
	event->accept();
}

void MainWindow::createStatusBar()
{	
	statusLabel_ = new QLabel("Ready");
	statusLabel_->setFixedWidth(scrollArea->width());
	statusLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	statusBar()->addWidget(statusLabel_, 1);

	coordinateUnderMouseLabel_ = new QLabel("XYZ = [-, -, -]");
	coordinateUnderMouseLabel_->setFixedWidth(410);
	coordinateUnderMouseLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	statusBar()->addWidget(coordinateUnderMouseLabel_, 1);

	QLabel* space1 = new QLabel;
	statusBar()->addWidget(space1, 1);

	int length = 250;
	numPointsLabel_ = new QLabel;
	numPointsLabel_->setFixedWidth(length);
	numPointsLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	statusBar()->addPermanentWidget(numPointsLabel_, 1);

	numFootPrintFacesLabel_= new QLabel;
	numFootPrintFacesLabel_->setFixedWidth(length);
	numFootPrintFacesLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	statusBar()->addPermanentWidget(numFootPrintFacesLabel_, 1);

	numReconstructionFacesLabel_ = new QLabel;
	numReconstructionFacesLabel_->setFixedWidth(length);
	numReconstructionFacesLabel_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	statusBar()->addPermanentWidget(numReconstructionFacesLabel_, 1);

	QLabel* space2 = new QLabel;
	statusBar()->addWidget(space2, 1);

	//////////////////////////////////////////////////////////////////////////

	progress_bar_ = new QProgressBar;
	progress_bar_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
	progress_bar_->setFixedWidth(500);
	statusBar()->addPermanentWidget(progress_bar_, 1);

	//////////////////////////////////////////////////////////////////////////

	updateStatusBar();
}
void MainWindow::setManualInputWeights(bool b) {
	if (b) {
		panelClick_->setVisible(false);
		panelManual_->setVisible(true);
	}
	else {
		panelClick_->setVisible(true);
		panelManual_->setVisible(false);
	}
}


void MainWindow::createToolBar()
{
	QAction* actionSaveReconstruction = new QAction(this);
	actionSaveReconstruction->setText("Save Reconstruction");
	connect(actionSaveReconstruction, SIGNAL(triggered()), this, SLOT(saveReconstruction()));

	QAction* actionSavePointCloud = new QAction(this);
	actionSavePointCloud->setText("Save Point Cloud");
	connect(actionSavePointCloud, SIGNAL(triggered()), this, SLOT(savePointCloud()));

	QAction* actionSaveFootPrint = new QAction(this);
	actionSaveFootPrint->setText("Save Foot Print");
	connect(actionSaveFootPrint, SIGNAL(triggered()), this, SLOT(saveFootPrint()));

	QMenu *menu = new QMenu();
	menu->addAction(actionSaveReconstruction);
	menu->addSeparator();
	menu->addAction(actionSavePointCloud);
	menu->addAction(actionSaveFootPrint);

	QToolButton* toolButton = new QToolButton();
	toolButton->setText("Save");
	toolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
	toolButton->setMenu(menu);
	toolButton->setPopupMode(QToolButton::InstantPopup);
	QIcon icon;
	icon.addFile(QStringLiteral(":/Resources/filesave.png"), QSize(), QIcon::Normal, QIcon::Off);
	toolButton->setIcon(icon);
	toolBarFile->insertWidget(actionSnapshot, toolButton);

	toolBarFile->insertSeparator(actionSnapshot);
}


void MainWindow::updateStatusBar()
{
	QString points = "#points: 0";
	QString footprintFaces = "#faces(footprint): 0";
	QString reconstructionFaces = "#faces(reconstruction): 0";

	if (mainCanvas_->pointSet()) {
		points = QString("#points: %1").arg(mainCanvas_->pointSet()->num_points());
	}
	if (mainCanvas_->footPrint()) {
		footprintFaces = QString("#faces(footprint): %1").arg(mainCanvas_->footPrint()->size_of_facets());
	}
	if (mainCanvas_->reconstruction()) {
		reconstructionFaces = QString("#faces(reconstruction): %1").arg(mainCanvas_->reconstruction()->size_of_facets());
	}

	numPointsLabel_->setText(points);
	numFootPrintFacesLabel_->setText(footprintFaces);
	numReconstructionFacesLabel_->setText(reconstructionFaces);
}

void MainWindow::showCoordinateUnderMouse(const vec3& p, bool found) {
	QString coordString = "XYZ = [-, -, -]";
	if (found)
		coordString = QString("XYZ = [%1, %2, %3]").arg(p.x).arg(p.y).arg(p.z);
	coordinateUnderMouseLabel_->setText(coordString);
}

void MainWindow::about()
{
#if defined (ENV_32_BIT)
	QString title = QMessageBox::tr("<h3>City3D (32-bit)</h3>");
#elif defined (ENV_64_BIT)
	QString title = QMessageBox::tr("<h3>City3D (64-bit)</h3>");
#else
	QString title = QMessageBox::tr("<h3>City3D</h3>");
#endif

#ifndef NDEBUG
	title += QMessageBox::tr(" (Debug Version)");
#endif

	QString text = QMessageBox::tr(
		"<p>This program implements our building reconstruction method described in the following paper:</p>"
		
		"<p>City3D: Large-scale Building Reconstruction from Airborne LiDAR Point Clouds<br>"
		"<span style=\"font-style:italic;\">Jin Huang, Jantien Stoter, Ravi Peters, and Liangliang Nan<br>"
		"--------------------------------------------------------------------------</p>"

		"<p>For comments, suggestions, or any issues, please contact me at:<br>"
		"<a href=\"mailto:J.Huang-1@tudelft.nl\">J.Huang-1@tudelft.nl</a>.</p>"
		"<p>@July.1, 2022</p>"
		);

	QMessageBox::about(this, "About City3D", title + text);
}

void MainWindow::readSettings()
{
	QSettings settings("LiangliangNan", "City3D");
	curDataDirectory_ = settings.value("currentDirectory").toString();	
}

void MainWindow::writeSettings()
{
	QSettings settings("LiangliangNan", "City3D");
	settings.setValue("currentDirectory", curDataDirectory_);
}


void MainWindow::setCurrentFile(const QString &fileName)
{
	curDataDirectory_ = fileName.left(fileName.lastIndexOf("/") + 1); // path includes "/"

	setWindowModified(false);

	QString shownName = "Untitled";
	if (!fileName.isEmpty())
		shownName = strippedName(fileName);

	setWindowTitle(tr("%1[*] - %2").arg(shownName).arg(tr("City3D")));
}


bool MainWindow::open()
{
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open file"), curDataDirectory_,
		tr("Supported Files (*.las *.laz *.geojson *.ply *.obj)\n"
			"Point Cloud (*.las *.laz *.ply)\n"
			"Foot Print (*.geojson *.obj)\n"
			"Mesh (*.obj)")
	);

	if (fileName.isEmpty())
		return false;

	return doOpen(fileName);
}


bool MainWindow::saveReconstruction()
{
	Map* reconstruction = canvas()->reconstruction();
	if (!reconstruction) {
		Logger::warn("-") << "reconstruction does not exist" << std::endl;
		return false;
	}

	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Reconstruction"), reconstructionMeshFileName_,
		tr("Mesh (*.obj)")
		);

	if (fileName.isEmpty())
		return false;

	bool success = MapIO::save(fileName.toStdString(), canvas()->reconstruction());
	if (success) {
		setCurrentFile(fileName);
		status_message("File saved", 500);
		return true;
	}
	else {
		status_message("Saving failed", 500);
		return false;
	}
}

bool MainWindow::saveFootPrint()
{
	Map* foot = canvas()->footPrint();
	if (!foot) {
		Logger::warn("-") << "foot print data does not exist" << std::endl;
		return false;
	}

	QString suggestedName = footPrintMeshFileName_;
	suggestedName.truncate(suggestedName.lastIndexOf("."));
	suggestedName.append("_transformed.obj");
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Foot Print"), suggestedName,
		tr("Mesh (*.obj)")
	);

	if (fileName.isEmpty())
		return false;

	bool success = MapIO::save(fileName.toStdString(), foot);
	if (success) {
		setCurrentFile(fileName);
		status_message("File saved", 500);
		return true;
	}
	else {
		status_message("Saving failed", 500);
		return false;
	}
}


bool MainWindow::savePointCloud()
{
	PointSet* pset = canvas()->pointSet();
	if (!pset) {
		Logger::warn() << "point cloud does not exist" << std::endl;
		return false;
	}

	QString suggestedName = pointCloudFileName_;
	suggestedName.truncate(suggestedName.lastIndexOf("."));
	suggestedName.append("_segmented_transformed.ply");
	QString fileName = QFileDialog::getSaveFileName(this,
		tr("Save Point Cloud"), suggestedName,
		tr("Point Cloud (*.ply)")
	);

	if (fileName.isEmpty())
		return false;

	bool success = PointSetIO::save(fileName.toStdString(), pset);
	if (success) {
		setCurrentFile(fileName);
		status_message("File saved", 500);
		return true;
	}
	else {
		status_message("Saving failed", 500);
		return false;
	}
}


bool MainWindow::doOpen(const QString &fileName)
{
	std::string name = fileName.toStdString();
	std::string ext = FileUtils::extension(name);
	String::to_lowercase(ext);

	Map* foot = nil;
	Map* result = nil;
	PointSet* pset = nil;
	if (ext == "geojson" || ext == "obj") {
		if (ext == "geojson") {
			MapSerializer_json json;
			const PointSet* pset = canvas()->pointSet();
			foot = json.read(name, pset, pset ? pset->offset() : vec3(0, 0, 0));
		}
		else  // obj
			foot = MapIO::read(name);

		if (foot) {
			footPrintMeshFileName_ = fileName;
			canvas()->setFootPrint(foot);
			reconstructionMeshFileName_ = footPrintMeshFileName_;
			int idx = fileName.lastIndexOf(".");
			reconstructionMeshFileName_.truncate(idx);
			reconstructionMeshFileName_.append(".obj");

			canvas()->fitScreen(foot->bbox());
		}
	}
	else if (ext == "las" || ext == "laz" || ext == "ply") {
		pset = PointSetIO::read(name);
		if (pset) {
			pointCloudFileName_ = fileName;
			canvas()->setPointSet(pset);

			reconstructionMeshFileName_ = pointCloudFileName_;
			int idx = fileName.lastIndexOf(".");
			reconstructionMeshFileName_.truncate(idx);
			reconstructionMeshFileName_.append(".obj");

			canvas()->fitScreen(pset->bbox());
		}
	}

	if (pset || foot || result) {
		updateStatusBar();
		setCurrentFile(fileName);
		setWindowTitle(tr("%1[*] - %2").arg(strippedName(fileName)).arg(tr("City3D")));
		status_message("File loaded", 500);

		if (pset || foot) {
			canvas()->setReconstruction(nil);
			resetRendering();
		}
		return true;
	} 	
	else {	
		status_message("Open failed", 500);
		return false;
	} 
}


QString MainWindow::strippedName(const QString &fullFileName)
{
	return QFileInfo(fullFileName).fileName();
}


void MainWindow::resetRendering() {
	checkBoxShowFootPrint->setChecked(true);
	checkBoxShowReconstruction->setChecked(true);

	wgtRender_->checkBoxPointSet->setChecked(true);
	wgtRender_->checkBoxSegments->setChecked(true);
	wgtRender_->checkBoxPerPointColor->setChecked(true);
	wgtRender_->checkBoxSurface->setChecked(true);
	wgtRender_->checkBoxPerFaceColor->setChecked(true);
	wgtRender_->checkBoxSharpEdges->setChecked(false);
}


LinearProgramSolver::SolverName MainWindow::active_solver() const {
#ifdef HAS_GUROBI
    std::cout << "using the Gurobi solver" << std::endl;
    return LinearProgramSolver::GUROBI;
#endif
    // default to SCIP
    std::cout << "using the SCIP solver" << std::endl;
    return LinearProgramSolver::SCIP;
}
