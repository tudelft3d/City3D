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
#include <QDebug>
#include <QSettings>
#include <QCloseEvent>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QMimeData>
#include <QComboBox>
#include <QMenu>

#include "main_window.h"
#include "paint_canvas.h"

#include "dlg/wgt_render.h"
#include "dlg/weight_panel_click.h"
#include "dlg/weight_panel_manual.h"

#include "../basic/file_utils.h"
#include "../model/map_builder.h"
#include "../model/map_io.h"
#include "../model/map_serializer_json.h"
#include "../model/point_set_io.h"
#include "../method/method_global.h"
#include "../basic/attribute_serializer.h"


MainWindow::MainWindow(QWidget *parent)
: QMainWindow(parent)
, curDataDirectory_(".")
{	
	setupUi(this);

	//////////////////////////////////////////////////////////////////////////

	const std::string app_path = FileUtils::executable();
	std::string log_path = app_path;
#ifdef __APPLE__
	// macOS may put the executable file in an application bundle, e.g., "PolyFit.app/Contents/MacOS/PolyFit"
	std::string::size_type pos = log_path.find(".app");
	if (pos != std::string::npos)
		log_path = log_path.substr(0, pos);
#endif
	log_path = FileUtils::parent_directory(log_path);
	std::string full_path_log_file = log_path + "/" + FileUtils::base_name(app_path) + ".log";

	Logger::initialize();
	Logger::instance()->register_client(this);
	Logger::instance()->set_value(Logger::LOG_REGISTER_FEATURES, "*"); // log everything
	Logger::instance()->set_value(Logger::LOG_FILE_NAME, full_path_log_file);
	//Logger::instance()->set_value("log_features",
	//	"EigenSolver;MapBuilder;MapParameterizer\
	//	LinearSolver");

	// Liangliang: added the time stamp in the log file
	std::string tstr = String::current_time();
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

	mainCanvas_ = new PaintCanvas(this);
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
	// ------------------------------------------------------

	// actions for open
	QAction* actionOpenPointCloud = new QAction(this);
	actionOpenPointCloud->setText("Open Point Cloud");
	connect(actionOpenPointCloud, SIGNAL(triggered()), this, SLOT(openPointCloud()));

	actionOpenFootPrint_ = new QAction(this);
	actionOpenFootPrint_->setText("Open Foot Print");
	connect(actionOpenFootPrint_, SIGNAL(triggered()), this, SLOT(openFootPrint()));

	QMenu* openMenu = new QMenu();
	openMenu->addAction(actionOpenPointCloud);
	openMenu->addAction(actionOpenFootPrint_);
	actionOpenFootPrint_->setEnabled(false);

	QToolButton* openToolButton = new QToolButton();
	openToolButton->setText("Open");
	openToolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
	openToolButton->setMenu(openMenu);
	openToolButton->setPopupMode(QToolButton::InstantPopup);
	QIcon openIcon;
	openIcon.addFile(QStringLiteral(":/Resources/fileopen.png"), QSize(), QIcon::Normal, QIcon::Off);
	openToolButton->setIcon(openIcon);

	toolBarFile->insertWidget(actionSnapshot, openToolButton);

	// ------------------------------------------------------

	// actions for save
	QAction* actionSaveReconstruction = new QAction(this);
	actionSaveReconstruction->setText("Save Reconstruction");
	connect(actionSaveReconstruction, SIGNAL(triggered()), this, SLOT(saveReconstruction()));

	QAction* actionSavePointCloud = new QAction(this);
	actionSavePointCloud->setText("Save Point Cloud");
	connect(actionSavePointCloud, SIGNAL(triggered()), this, SLOT(savePointCloud()));

	QAction* actionSaveFootPrint = new QAction(this);
	actionSaveFootPrint->setText("Save Foot Print");
	connect(actionSaveFootPrint, SIGNAL(triggered()), this, SLOT(saveFootPrint()));

	QMenu *saveMenu = new QMenu();
	saveMenu->addAction(actionSaveReconstruction);
	saveMenu->addSeparator();
	saveMenu->addAction(actionSavePointCloud);
	saveMenu->addAction(actionSaveFootPrint);

	QToolButton* saveToolButton = new QToolButton();
	saveToolButton->setText("Save");
	saveToolButton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
	saveToolButton->setMenu(saveMenu);
	saveToolButton->setPopupMode(QToolButton::InstantPopup);
	QIcon saveIcon;
	saveIcon.addFile(QStringLiteral(":/Resources/filesave.png"), QSize(), QIcon::Normal, QIcon::Off);
	saveToolButton->setIcon(saveIcon);

	toolBarFile->insertWidget(actionSnapshot, saveToolButton);
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

bool MainWindow::want_footprint() {
    auto reply = QMessageBox::question(this, "Generate footprint?", "No footprint provided, generate one?",
                                  QMessageBox::No|QMessageBox::Yes);
    if (reply == QMessageBox::Yes) {
        Logger::out("-") << "we will generate a footprint..." << std::endl;
		return true;
    } else {
		Logger::out("-") << "no footprint will be generated (reconstruction cannot continue)" << std::endl;
		return false;
    }
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
		"<p>@July 1, 2022</p>"
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


bool MainWindow::openPointCloud() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open point cloud file"), curDataDirectory_,
		tr("Point Cloud (*.las *.laz *.ply)")
	);

	if (fileName.isEmpty())
		return false;

	return doOpen(fileName);
}


bool MainWindow::openFootPrint() {
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open foot print file"), curDataDirectory_,
		tr("Foot Print (*.geojson *.obj)")
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

	Map* footprint = nil;
	Map* result = nil;
	PointSet* pset = nil;
	if (ext == "geojson" || ext == "obj") {
        const PointSet* pset = canvas()->pointSet();
        if (!pset) {
            Logger::err("-") << "point cloud doesn't exist. Please load point cloud first" << std::endl;
            return false;
        }

        /// ToDo: in this demo the Z coordinate of the footprint/ground is set to the min_Z of the point cloud.
        ///       This is not optimal (at least noise, outliers, and incompleteness not considered).
        ///       In practice, the Z coordinate of each building should be determined by extracting local ground planes,
        ///       or directly from available DTM or DSM data.
        const vec3& offset = pset->offset();
        footprint = MapIO::read(name, vec3(offset.x, offset.y, -pset->bbox().z_min()));
		if (footprint) {
			footPrintMeshFileName_ = fileName;
            footprint->set_name(fileName.toStdString());
			canvas()->setFootPrint(footprint);
			reconstructionMeshFileName_ = footPrintMeshFileName_;
			int idx = fileName.lastIndexOf(".");
			reconstructionMeshFileName_.truncate(idx);
			reconstructionMeshFileName_.append(".obj");

			canvas()->fitAll();
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
			actionOpenFootPrint_->setEnabled(true);
			canvas()->fitAll();
		}
	}

	if (pset || footprint || result) {
		updateStatusBar();
		setCurrentFile(fileName);
		setWindowTitle(tr("%1[*] - %2").arg(strippedName(fileName)).arg(tr("City3D")));
		status_message("File loaded", 500);

		if (pset || footprint) {
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
	Logger::out("-") << "using the Gurobi solver" << std::endl;
    return LinearProgramSolver::GUROBI;
#else
    // default to SCIP
	Logger::out("-") << "using the SCIP solver" << std::endl;
    return LinearProgramSolver::SCIP;
#endif
}
