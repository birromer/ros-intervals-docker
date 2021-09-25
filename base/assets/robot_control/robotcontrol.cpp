#include "robotcontrol.h"

RobotControl::RobotControl()
{
	p = new Protocol;

	centralWidget = new QWidget;
	centralLayout = new QGridLayout;

    settings = new QSettings("settings.ini",QSettings::IniFormat);

	view = new QQuickWidget;
	view->rootContext()->setContextProperty("rbctrl", this);
    view->setSource(QUrl::fromLocalFile("mapview.qml"));
	view->setResizeMode(QQuickWidget::SizeRootObjectToView);
	view->resize(500,500);
	item = view->rootObject();

	edit = new QTextEdit;
	edit->setReadOnly(true);
	
	centralLayout->addWidget(view,0,0,8,6);

	cmdLayout  = new QVBoxLayout;
	mission = new QGroupBox("Mission");
	missionLayout = new QVBoxLayout;

	tabMission = new QTabWidget;

	tabMission->addTab(edit,"Log");

	waypointsMission = new QWidget;
	waypointsMissionLayout = new QFormLayout;

	linesWaypointsMission = new QCheckBox;
	waypointsMissionLayout->addRow("Line following mode",linesWaypointsMission);
	listOfWaypoints = new QListWidget;
	waypointsMissionLayout->addRow(listOfWaypoints);

	buttonsWaypointsMission1 = new QHBoxLayout;
	startSelectWaypoints = new QPushButton("Add waypoints");
	stopSelectWaypoints = new QPushButton("Stop selection");
	clearWaypoints = new QPushButton("Clear");
	moveWaypoint = new QPushButton("Move");
	buttonsWaypointsMission1->addWidget(startSelectWaypoints);
	buttonsWaypointsMission1->addWidget(stopSelectWaypoints);
	buttonsWaypointsMission1->addWidget(clearWaypoints);
	buttonsWaypointsMission1->addWidget(moveWaypoint);
	
	buttonsWaypointsMission2 = new QHBoxLayout;
	waypointsMissionSend = new QPushButton("Send to robot");
	waypointsMissionLoad = new QPushButton("Load from robot");


	buttonsWaypointsMission2->addWidget(waypointsMissionSend);
	buttonsWaypointsMission2->addWidget(waypointsMissionLoad);


	waypointsMissionLayout->addRow(buttonsWaypointsMission1);
	waypointsMissionLayout->addRow(buttonsWaypointsMission2);
	waypointsMission->setLayout(waypointsMissionLayout);
	tabMission->addTab(waypointsMission,"Waypoints selection");

	robotMission = new QWidget;
	robotMissionLayout = new QVBoxLayout;

	speedBox = new QGroupBox("Speed");
	speedLayout = new QVBoxLayout;
	speedSetLayout = new QHBoxLayout; 
	speed = new QSlider(Qt::Horizontal);
	speed->setMinimum(0);
	speed->setMaximum(100);
	speedValue = new QLCDNumber;
	speedValue->setSegmentStyle(QLCDNumber::Flat);
	speedButtonLayout = new QHBoxLayout;
	setSpeed = new QPushButton("Set current speed");
	getSpeed = new QPushButton("Get current speed");

	speedButtonLayout->addWidget(getSpeed);
	speedButtonLayout->addWidget(setSpeed);
	speedSetLayout->addWidget(speed);
	speedSetLayout->addWidget(speedValue);
	speedLayout->addLayout(speedSetLayout);
	speedLayout->addLayout(speedButtonLayout);
	speedBox->setLayout(speedLayout);

	robotMissionLayout->addWidget(speedBox);
	missionWp = new QGroupBox("Waypoints mission"); 
	missionWpLayout = new QHBoxLayout;

	startRobotMission = new QPushButton("Start mission");
	missionWpLayout->addWidget(startRobotMission);
	missionWp->setLayout(missionWpLayout);

	missionHeading = new QGroupBox("Heading mission"); 
	headingMissionLayout = new QFormLayout;
	headingHeadingMission = new QDoubleSpinBox;
	headingHeadingMission->setSuffix(" deg");
	headingHeadingMission->setRange(0.,360.);
	robotMissionButtonsLayout2 = new QHBoxLayout;
	headingMissionStart = new QPushButton("Start following heading");
	headingMissionLayout->addRow("Heading",headingHeadingMission);
	robotMissionButtonsLayout2->addWidget(headingMissionStart);

	headingMissionLayout->addRow(robotMissionButtonsLayout2);
	missionHeading->setLayout(headingMissionLayout);

	stopRobot = new QPushButton("Stop");

	robotMissionLayout->addWidget(missionWp);
	robotMissionLayout->addWidget(missionHeading);
	robotMissionLayout->addWidget(stopRobot);


	robotMission->setLayout(robotMissionLayout);
	tabMission->addTab(robotMission,"Robot mission");

	other = new QWidget;
	otherLayout = new QFormLayout;
	cog = new QLabel;
	sog = new QLabel;
	distToLine = new QLabel;
	distToWp = new QLabel;
	mode = new QLabel;
	headingObj = new QLabel;
	distToEnd = new QLabel;
	timeSinceStart = new QLabel;

	otherLayout->addRow("Mode",mode);
	otherLayout->addRow("<font color=\"purple\">Course over ground</font>",cog);
	otherLayout->addRow("<font color=\"blue\">Heading command</font>",headingObj);
	otherLayout->addRow("Speed over ground",sog);
	otherLayout->addRow("Distance to waypoint",distToWp);
	otherLayout->addRow("Distance to last waypoint",distToEnd);
	otherLayout->addRow("Distance to line",distToLine);
	otherLayout->addRow("Mission duration ",timeSinceStart);
	
	for(int i = 0; i < 4; i++)
	{
		QLabel *customLabel = new QLabel;
		QString customLabelName = settings->value("custom/message"+QString::number(i)).toString();
		if(customLabelName != "")
		{
			otherLayout->addRow(customLabelName,customLabel);
		}
		customUnits.append(settings->value("custom/unitMessage"+QString::number(i)).toString());
		customLabels.append(customLabel);
	}

	other->setLayout(otherLayout);
	tabMission->addTab(other,"Other");

	survey = new QWidget;
	surveyLayout = new QFormLayout;
	loadLastWp = new QPushButton("Load last waypoint");
	lastWp = new QLabel;
	surveyLength = new QDoubleSpinBox;
	surveyNumberOfRails = new QDoubleSpinBox;
	surveyWidth = new QDoubleSpinBox;
	surveyYaw = new QDoubleSpinBox;
	surveyGenerate = new QPushButton("Generate");

	surveyLength->setValue(15);
	surveyWidth->setValue(10);
	surveyNumberOfRails->setValue(5);
	surveyYaw->setRange(-360.,360.);

	surveyLayout->addRow(loadLastWp);
	surveyLayout->addRow("Waypoint",lastWp);
	surveyLayout->addRow("Length",surveyLength);
	surveyLayout->addRow("Width",surveyWidth);
	surveyLayout->addRow("Number of double rails",surveyNumberOfRails);
	surveyLayout->addRow("Yaw",surveyYaw);
	surveyLayout->addRow(surveyGenerate);

	surveyGenerate->setEnabled(false);

	survey->setLayout(surveyLayout);

	tabMission->addTab(survey,"Survey");


	missionLayout->addWidget(tabMission);
	mission->setLayout(missionLayout);

	cmdLayout->addWidget(mission);


	status  = new QGroupBox("Status");
	statusLayout = new QVBoxLayout;

	pos = new QGroupBox("Position");
	posLayout = new QFormLayout;
	lat = new QLabel;
	lon = new QLabel;
	alt = new QLabel;
	error_xy = new QLabel;
	error_alt = new QLabel;

	posLayout->addRow("Latitude",lat);
	posLayout->addRow("Longitude",lon);
	posLayout->addRow("Altitude",alt);
	posLayout->addRow("Error lat/lon",error_xy);
	posLayout->addRow("Error altitude",error_alt);
	pos->setLayout(posLayout);
	statusLayout->addWidget(pos);

	attitude  = new QGroupBox("Attitude");
	attitudeLayout = new QFormLayout;
	heading = new QLabel;
	pitch = new QLabel;
	roll = new QLabel;
	attitudeLayout->addRow("<font color=\"green\">Heading</font>",heading);
	attitudeLayout->addRow("Pitch",pitch);
	attitudeLayout->addRow("Roll",roll);
	attitude->setLayout(attitudeLayout);
	statusLayout->addWidget(attitude);

	status->setLayout(statusLayout);
	cmdLayout->addWidget(status);

	centralLayout->addLayout(cmdLayout,0,6,8,2);
	//centralLayout->addLayout(cmdLayout);

	centralWidget->setLayout(centralLayout);
	setCentralWidget(centralWidget);

	statusBar = new QStatusBar;
	setStatusBar(statusBar);

	menus = new QMenuBar;

	menuFile = new QMenu("File");
	actionOpen =  menuFile->addAction("Open mission");
	actionSave =  menuFile->addAction("Save mission");
	actionExit =  menuFile->addAction("Exit");

	menuFile->addAction(actionOpen);
	menuFile->addAction(actionSave);
	menuFile->addAction(actionExit);

	menus->addMenu(menuFile);



	menuSerial = new QMenu("Connection to robot");
	menuPort = new QMenu("Port");
	

	actionCustom = menuPort->addAction("Custom");
	actionCustom->setCheckable(true);
	actionCustom->setChecked(true);
	actionsPort = new QActionGroup(this);
	actionsPort->setExclusive(true);
	actionsPort->addAction(actionCustom);

	customPort = new QLineEdit("/dev/ttyUSB1");
	actionCustomPort = new QWidgetAction(this);
	actionCustomPort->setDefaultWidget(customPort);
	menuPort->addAction(actionCustomPort);
	menuSerial->addMenu(menuPort);


	menuBaudrate = new QMenu("Baudrate");
    actionsBaudrate = new QActionGroup(this);
	b115200 = menuBaudrate->addAction("115200");
	b57600 = menuBaudrate->addAction("57600");
	b38400 = menuBaudrate->addAction("38400");
	b19200 = menuBaudrate->addAction("19200");
	b9600 = menuBaudrate->addAction("9600");
	b921600 = menuBaudrate->addAction("921600");
	b115200->setCheckable(true);
	b115200->setChecked(true);
	b57600->setCheckable(true);
	b38400->setCheckable(true);
	b19200->setCheckable(true);
	b9600->setCheckable(true);
	b921600->setCheckable(true);
	actionsBaudrate->addAction(b115200);
	actionsBaudrate->addAction(b57600);
	actionsBaudrate->addAction(b38400);
	actionsBaudrate->addAction(b19200);
	actionsBaudrate->addAction(b9600);
	actionsBaudrate->addAction(b921600);
	actionsBaudrate->setExclusive(true);
	menuSerial->addMenu(menuBaudrate);

	actionConnect =  menuSerial->addAction("Connect");
	actionDisconnect =  menuSerial->addAction("Disconnect");
	menus->addMenu(menuSerial);

	menuConfig = new QMenu("Configuration");

	menuWaypointsSize = new QMenu("Waypoints size");

	waypointsSize = new QSpinBox;
	waypointsSize->setRange(1,10);
	waypointsSize->setValue(3);

	actionWaypointsSize = new QWidgetAction(this);
	actionWaypointsSize->setDefaultWidget(waypointsSize);

	menuWaypointsSize->addAction(actionWaypointsSize);
	menuConfig->addMenu(menuWaypointsSize);

	actionClearLog = menuConfig->addAction("Clear log");
	actionClearTrace = menuConfig->addAction("Clear trace");
	actionShowCorrectMessages = menuConfig->addAction("Print correct messages in log");
	actionShowTrace = menuConfig->addAction("Show trace");
	actionShowTrace->setCheckable(true);
	actionShowTrace->setChecked(true);
	actionShowCorrectMessages->setCheckable(true);
	menus->addMenu(menuConfig);

	actionCenterOnRobot = menus->addAction("Center on robot");

	setMenuBar(menus);
	setWindowTitle("USV control");

	timerListSerial = new QTimer;
	timerListSerial->start(1000);

	timerStatus = new QTimer;
	timerStatus->start(1000);

	Q_EMIT drawLineSignal(QGeoCoordinate(0,0),QGeoCoordinate(0,0),1,"robot","green");
	robot = item->findChild<QObject*>("robot");

	Q_EMIT drawLineSignal(QGeoCoordinate(0,0),QGeoCoordinate(0,0),1,"cross","purple");
	cross = item->findChild<QObject*>("cross");

	Q_EMIT drawLineSignal(QGeoCoordinate(0,0),QGeoCoordinate(0,0),1,"cogline","purple");
	cogLine = item->findChild<QObject*>("cogline");

	Q_EMIT drawLineSignal(QGeoCoordinate(0,0),QGeoCoordinate(0,0),1,"headingobj","blue");
	headingObjLine = item->findChild<QObject*>("headingobj");

	settingWaypoints = false;
	movingWaypoint = false;
	counter_waypoints = 0;
	counter_lines = 0;
	counter_pointsTrace = 0;
	counter_messages = 0;
	bytesReceivedPerSec = 0;
	bytesSentPerSec = 0;
	startSelectWaypoints->setEnabled(true);
	stopSelectWaypoints->setEnabled(false);
	clearWaypoints->setEnabled(true);
	waypointsMissionSend->setEnabled(false);
	waypointsMissionLoad->setEnabled(false);
	actionSave->setEnabled(true);
	actionOpen->setEnabled(true);



	waypointsSizeValue = waypointsSize->value();


	timerSerial = new QTimer;
	actionConnect->setEnabled(true);
	actionDisconnect->setEnabled(false);

	posReceived = false;
	overGroundReceived = false;
	attitudeReceived = false;
	speedReceived = false;
	missionReceived = false;
	currentWpReceived = false;
	distToLineReceived = false;
	hidingTrace = false;
	headingObjReceived = false;
	customReceived = false;

	actionCenterOnRobot->setEnabled(posReceived);

	C = proj_context_create();
	setProj();

	timerSendObjectives = new QTimer;
	objectivesToSendId = 0;
	numberOfObjectivesToReceive = -1;


    connect(item, SIGNAL(sendCoordinates(double,double,bool)),this, SLOT(callbackWaypoint(double,double,bool)));
    connect(item, SIGNAL(sendCoordinatesDoubleClick(double,double,double,bool)),this, SLOT(callbackDoubleClick(double,double,double,bool)));
    connect(startSelectWaypoints, SIGNAL(clicked()),this, SLOT(setWaypoints()));
    connect(stopSelectWaypoints, SIGNAL(clicked()),this, SLOT(stopSettingWaypoints()));
    connect(clearWaypoints, SIGNAL(clicked()),this, SLOT(clearAllWaypoints()));
	connect(moveWaypoint, SIGNAL(clicked()),this, SLOT(setMovingWaypointMode()));
    connect(actionClearLog, SIGNAL(triggered()),edit, SLOT(clear()));
    connect(speed, SIGNAL(valueChanged(int)), speedValue, SLOT(display(int)));
    connect(waypointsSize, SIGNAL(valueChanged(int)),this, SLOT(changeWaypointsSize(int)));   
    connect(actionConnect, SIGNAL(triggered()),this, SLOT(serialConnect()));
    connect(actionDisconnect, SIGNAL(triggered()),this, SLOT(serialDisconnect()));
    connect(timerSerial, SIGNAL(timeout()),this, SLOT(serialRead()));
    connect(actionCenterOnRobot, SIGNAL(triggered()),this, SLOT(centerOnRobot()));
    connect(waypointsMissionSend, SIGNAL(clicked()),this, SLOT(sendWaypoints()));
	connect(setSpeed, SIGNAL(clicked()),this, SLOT(sendSetSpeed()));
	connect(getSpeed, SIGNAL(clicked()),this, SLOT(sendGetSpeed()));
	connect(waypointsMissionLoad, SIGNAL(clicked()),this, SLOT(sendGetWaypoints()));
	connect(startRobotMission, SIGNAL(clicked()),this, SLOT(sendRobotStart()));
	connect(stopRobot, SIGNAL(clicked()),this, SLOT(sendRobotStop()));
	connect(headingMissionStart, SIGNAL(clicked()),this, SLOT(sendRobotHeadingStart()));
	connect(actionSave, SIGNAL(triggered()),this, SLOT(saveCurrentMission()));
	connect(actionOpen, SIGNAL(triggered()),this, SLOT(loadMission()));
	connect(actionClearTrace, SIGNAL(triggered()),this, SLOT(clearTrace()));
	connect(actionShowTrace, SIGNAL(triggered(bool)),this, SLOT(hideTraceChangedStatus(bool)));
	connect(timerStatus, SIGNAL(timeout()),this, SLOT(setStatusMessage()));
	connect(timerListSerial, SIGNAL(timeout()),this, SLOT(addSerialPortsToMenu()));
	connect(actionExit, SIGNAL(triggered()),qApp, SLOT(quit()));
	connect(timerSendObjectives, SIGNAL(timeout()),this, SLOT(sendObjectives()));
	connect(loadLastWp, SIGNAL(clicked()),this, SLOT(setWaypointSurvey()));
	connect(surveyGenerate, SIGNAL(clicked()),this, SLOT(generateSurvey()));


	QObject *map_ = item->findChild<QObject*>("map");
	if(map_ )
	{	
		map_ ->setProperty("zoomLevel", 18);
		QQmlProperty property_lat(map_ , "center.latitude");
		QQmlProperty property_lon(map_ , "center.longitude");
		property_lat.write(settings->value("pos_init/lat",48.40181351).toDouble());
		property_lon.write(settings->value("pos_init/lon",-4.519093024).toDouble());
	}
}

void RobotControl::latLonToXY(double lat_, double lon_, double &x_, double &y_)
{
    a = proj_coord(lon_,lat_,0,0);
   	b = proj_trans(P, PJ_FWD, a);
    x_ = b.enu.n;
    y_ = b.enu.e;
}

void RobotControl::xyToLatLon(double x_, double y_,double &lat_, double &lon_)
{
   	a = proj_coord(y_,x_,0,0);
   	b = proj_trans (P, PJ_INV, a);
	lat_ = b.lp.phi;
	lon_ = b.lp.lam;
}

void RobotControl::setProj(double lon)
{
	int zone = std::floor((lon + 180.0) / 6) + 1;
	std::string s = "+proj=utm +zone="+std::to_string(zone)+" +datum=WGS84";

	P = proj_create_crs_to_crs(C,"EPSG:4326",s.c_str(), NULL);
	P_for_GIS = proj_normalize_for_visualization(C, P);
	proj_destroy(P);
	P = P_for_GIS;

}

void RobotControl::addSerialPortsToMenu()
{

	if(menuSerial->activeAction() != menuPort->menuAction())
	{
		std::vector<serial::PortInfo> portList = serial::list_ports();

		QString checkedName = "";
		QAction *checkedAction = actionsPort->checkedAction();
		if(checkedAction != nullptr)
		{
			checkedName = checkedAction->text();
		}
		QList<QAction *> actions = actionsPort->actions();
		for(int i = 0; i < actions.size(); i++)
		{
			if(actions[i]->text() != "Custom")
			{
				actionsPort->removeAction(actions[i]);
				menuPort->removeAction(actions[i]);
				delete actions[i];
			}
		}
		for(int i = 0; i < portList.size(); i++)
		{
			QAction *actionPort = menuPort->addAction(QString::fromStdString(portList[i].port));
			actionPort->setCheckable(true);
			actionsPort->addAction(actionPort);
			if(checkedName == actionPort->text())
			{
				actionPort->setChecked(true);
			}
		}
		if(checkedName == "")
		{
			actionCustom->setChecked(true);
		}
	}
	timerListSerial->start(1000);
}

void RobotControl::setStatusMessage()
{
	statusBar->showMessage("Messages received: " + QString::number(counter_messages) + "   Bytes received per second: " + QString::number(bytesReceivedPerSec) + "   Bytes sent per second: " + QString::number(bytesSentPerSec),1000);
	bytesReceivedPerSec = 0;
	bytesSentPerSec = 0;
	timerStatus->start(1000);
}

void RobotControl::setWaypointSurvey()
{
	int s = waypoints.size();
	if(s != 0)
	{
		wpSurveyLat = waypoints[s-1].lat;
		wpSurveyLon = waypoints[s-1].lon;
		surveyGenerate->setEnabled(true);
		lastWp->setText(QString::number(wpSurveyLon,'f',8) + "  " + QString::number(wpSurveyLat,'f',8));
	}

}

void RobotControl::generateSurvey()
{
	double x0,y0;

	latLonToXY(wpSurveyLat,wpSurveyLon,x0,y0);

	std::vector<double> vx = {x0};
	std::vector<double> vy = {y0};
	std::vector<bool> vline = {true};

	double lengthOfRail = surveyLength->value();
	double yaw = surveyYaw->value()*M_PI/180.;
	double nb_rails = surveyNumberOfRails->value();
	double width = surveyWidth->value();

	double x_ = x0;
	double y_ = y0;
	for (int i = 0; i < nb_rails; i++)
	{
		x_ = x_ + lengthOfRail*cos(yaw);
		y_ = y_ + lengthOfRail*sin(yaw);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(true);
		x_ = x_ + width/2.*cos(yaw+M_PI/4.);
		y_ = y_ + width/2.*sin(yaw+M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ + width/2.*cos(yaw+3.*M_PI/4.);
		y_ = y_ + width/2.*sin(yaw+3.*M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ - lengthOfRail*cos(yaw);
		y_ = y_ - lengthOfRail*sin(yaw);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(true);
		x_ = x_ - width/2.*cos(yaw-M_PI/4.);
		y_ = y_ - width/2.*sin(yaw-M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ + width/2.*cos(yaw+M_PI/4.);
		y_ = y_ + width/2.*sin(yaw+M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
	}

	int u = std::floor(lengthOfRail/width);
	x_ = x_ + width/2.*cos(yaw-M_PI/4.);
	y_ = y_ + width/2.*sin(yaw-M_PI/4.);
	vx.push_back(x_);
	vy.push_back(y_);
	vline.push_back(false);

	for(int i = 0; i < u; i++)
	{
		x_ = x_ + (nb_rails)*width*sqrt(2.)*cos(yaw-M_PI/2.);
		y_ = y_ + (nb_rails)*width*sqrt(2.)*sin(yaw-M_PI/2.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(true);
		x_ = x_ + width/2.*cos(yaw-M_PI/4.);
		y_ = y_ + width/2.*sin(yaw-M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ + width/2.*cos(yaw+M_PI/4.);
		y_ = y_ + width/2.*sin(yaw+M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ - (nb_rails)*width*sqrt(2.)*cos(yaw-M_PI/2.);
		y_ = y_ - (nb_rails)*width*sqrt(2.)*sin(yaw-M_PI/2.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(true);
		x_ = x_ - width/2.*cos(yaw-3.*M_PI/4.);
		y_ = y_ - width/2.*sin(yaw-3.*M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
		x_ = x_ + width/2.*cos(yaw-M_PI/4.);
		y_ = y_ + width/2.*sin(yaw-M_PI/4.);
		vx.push_back(x_);
		vy.push_back(y_);
		vline.push_back(false);
	}

	double lat_,lon_;
	clearAllWaypoints();
	bool statusCheckbox = linesWaypointsMission->isChecked();
	settingWaypoints = true;
	for(int i = 0; i < vx.size() -2; i++)
	{
		linesWaypointsMission->setChecked(vline[i]);
		xyToLatLon(vx[i],vy[i],lat_,lon_);
		callbackWaypoint(lat_,lon_,true);	
	}
	linesWaypointsMission->setChecked(statusCheckbox);
	settingWaypoints = false;
}

void RobotControl::callbackWaypoint(double lat, double lon, bool isLeft)
{
	if(settingWaypoints and isLeft)
	{	
		counter_waypoints++;
		bool isLineMode = linesWaypointsMission->isChecked() and waypoints.size() >= 1;
		QListWidgetItem *elem = new QListWidgetItem;
		waypoint wp;
		wp.lat = lat;
		wp.lon = lon ;
		wp.elem = elem;
		wp.name = "Waypoint_" + QString::number(counter_waypoints);
		wp.lineMode = isLineMode;
		if(isLineMode)
		{
			elem->setText(QString::number(lat,'g',12)+","+QString::number(lon,'g',12) + " (line)");
			Q_EMIT drawCircleSignal(QGeoCoordinate(lat,lon),waypointsSizeValue,wp.name,"blue");
		}
		else if(waypoints.size() == 0)
		{
			elem->setText(QString::number(lat,'g',12)+","+QString::number(lon,'g',12));
			Q_EMIT drawCircleSignal(QGeoCoordinate(lat,lon),waypointsSizeValue,wp.name,"green");
		}
		else
		{
			elem->setText(QString::number(lat,'g',12)+","+QString::number(lon,'g',12));
			Q_EMIT drawCircleSignal(QGeoCoordinate(lat,lon),waypointsSizeValue,wp.name,"red");
		}
		listOfWaypoints->addItem(elem);

		if(waypoints.size() != 0)
		{
			counter_lines++;
			line l;
			l.wp1Name = waypoints.last().name;
			l.wp2Name = wp.name;
			l.name = "line" + QString::number(counter_lines);
			wp.lineName = l.name;
			if(isLineMode)
			{
				Q_EMIT drawLineSignal(QGeoCoordinate(waypoints.last().lat,waypoints.last().lon),QGeoCoordinate(wp.lat,wp.lon),3,l.name,"blue");
			}
			else
			{
				Q_EMIT drawLineSignal(QGeoCoordinate(waypoints.last().lat,waypoints.last().lon),QGeoCoordinate(wp.lat,wp.lon),2,l.name,"red");
			}
			lines.append(l);
		}
		else
		{
			wp.lineName = "none";
		}
		
		waypoints.append(wp);
	}
	else if(movingWaypoint and isLeft)
	{

		waypoints[waypointToMoveId].lat = lat;
		waypoints[waypointToMoveId].lon = lon;
		if(waypoints[waypointToMoveId].lineMode)
		{
			waypoints[waypointToMoveId].elem->setText(QString::number(lat,'g',12)+","+QString::number(lon,'g',12) + " (line)");
		}
		else
		{
			waypoints[waypointToMoveId].elem->setText(QString::number(lat,'g',12)+","+QString::number(lon,'g',12));
		}

		startSelectWaypoints->setEnabled(true);
		stopSelectWaypoints->setEnabled(false);
		clearWaypoints->setEnabled(true);
		moveWaypoint->setEnabled(true);
		waypointsMissionSend->setEnabled(true);
		movingWaypoint = false;
		edit->append("Moved to " + QString::number(lat,'g',12) + ", " + QString::number(lon,'g',12));	
		dynamicReconfigureWaypoints();
		dynamicReconfigureLines();
	}
	else if(isLeft and waypoints.size() > 0)
	{
		int closestWpId = -1;
		double bestDistance = HUGE_VAL;
		double x0,y0,x1,y1;
		latLonToXY(lat,lon,x0,y0);
		for(int i = 0; i < waypoints.size(); i++)
		{
			latLonToXY(waypoints[i].lat,waypoints[i].lon,x1,y1);
			double distance = sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));
			if(distance < bestDistance)
			{
				bestDistance = distance;
				closestWpId = i;
			}
		}
		if(closestWpId != -1)
		{
			listOfWaypoints->setCurrentItem(waypoints[closestWpId].elem);
		}
	}


}

void RobotControl::clearAllWaypoints()
{
	for(int i = 0; i < waypoints.size(); i++)
	{
			QObject *circle = item->findChild<QObject*>(waypoints[i].name);
			if(circle)
			{	
		    	circle->setProperty("visible", false);
		    	if(waypoints[i].lineName != "none")
		    	{
		    		QObject *line = item->findChild<QObject*>(waypoints[i].lineName);
		    		if(line)
		    		{
		    			line->setProperty("visible",false);
		    		}
	    		}
			}
			listOfWaypoints->removeItemWidget(waypoints[i].elem);
			delete waypoints[i].elem;
	}
	waypoints.clear();
	lines.clear();
}

void RobotControl::hideTraceChangedStatus(bool i)
{
	hidingTrace = !actionShowTrace->isChecked();
	for(int i = 0; i < pointsTrace.size(); i++)
	{
		QObject *circle = item->findChild<QObject*>(pointsTrace[i]);
		if(circle)
		{	
	    	circle->setProperty("visible", !hidingTrace);
		}
	}	
}

void RobotControl::clearTrace()
{
	for(int i = 0; i < pointsTrace.size(); i++)
	{
			QObject *circle = item->findChild<QObject*>(pointsTrace[i]);
			if(circle)
			{	
		    	circle->setProperty("visible", false);
			}
	}
	pointsTrace.clear();
}

void RobotControl::loadMission()
{
	QString fileName = QFileDialog::getOpenFileName(this, "Open mission","", "Robot mission (*.rbm)",0,QFileDialog::DontUseNativeDialog);
    if(fileName != "")
    {
		QFile file(fileName);
		if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		{
			edit->append("Failure to open " + fileName);
			return;
		}
		clearAllWaypoints();
		bool statusCheckbox = linesWaypointsMission->isChecked();
		settingWaypoints = true;

		QTextStream in(&file);
		while (!in.atEnd())
		{
			QStringList elems = in.readLine().split(";");
			if(elems.size() == 3)
			{
				bool valid1, valid2, valid3;
				double latwp = elems[0].toDouble(&valid1);
				double lonwp = elems[1].toDouble(&valid2);
				int lineModeWp = elems[2].toInt(&valid3);
				if(valid1 and valid2 and valid3)
				{
					linesWaypointsMission->setChecked((bool)lineModeWp);
					callbackWaypoint(latwp,lonwp,true);
				}
				else
				{
					edit->append("Incorrect data");
					break;
				}
			}
			else
			{
				edit->append("Incorrect data");
				break;
			}	
		}
		linesWaypointsMission->setChecked(statusCheckbox);
		settingWaypoints = false;
		file.close();
    }
}

void RobotControl::saveCurrentMission()
{
    QString fileName = QFileDialog::getSaveFileName(this, "Save mission","", "Robot mission (*.rbm)",0,QFileDialog::DontUseNativeDialog);
    if(fileName != "")
    {
    	if(!fileName.endsWith(".rbm",Qt::CaseInsensitive))
    	{
    		fileName += ".rbm";
    	}
		QFile file(fileName);
		if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			return;
		}

		QTextStream out(&file);
		out.setRealNumberPrecision(12);
		for(int i = 0; i < waypoints.size(); i++)
		{
			out << waypoints[i].lat << ";" << waypoints[i].lon << ";" << waypoints[i].lineMode << "\n";
		}
		file.close();
		edit->append("Mission saved");
    }
}

void RobotControl::centerOnRobot()
{
	QObject *map_ = item->findChild<QObject*>("map");
	if(map_ )
	{	
		map_ ->setProperty("zoomLevel", 18);
		QQmlProperty property_lat(map_ , "center.latitude");
		QQmlProperty property_lon(map_ , "center.longitude");
		property_lat.write(currentPos.lat);
		property_lon.write(currentPos.lon);
	}
}

void RobotControl::callbackDoubleClick(double lat, double lon, double zoomLevel, bool forward)
{
	QObject *map_ = item->findChild<QObject*>("map");
	if(map_ and !movingWaypoint and !settingWaypoints)
	{	
		if(forward)
		{
			map_->setProperty("zoomLevel", std::min((int)zoomLevel + 1,20));
		}
		else
		{
			map_->setProperty("zoomLevel", std::max((int)zoomLevel - 1,2));
		}
		QQmlProperty property_lat(map_ , "center.latitude");
		QQmlProperty property_lon(map_ , "center.longitude");
		property_lat.write(lat);
		property_lon.write(lon);
	}

}

void RobotControl::serialConnect()
{
	actionConnect->setEnabled(false);
	actionDisconnect->setEnabled(true);
	waypointsMissionSend->setEnabled(true);
	waypointsMissionLoad->setEnabled(true);
	bytesRemaining = -1;
	messageType = 0x00;
	buffer.clear();
	if(actionsBaudrate->checkedAction() == nullptr or actionsPort->checkedAction() == nullptr)
	{
		edit->append("No port or baudrate selected");
	}
	else
	{
		int baudrate = actionsBaudrate->checkedAction()->text().toInt();
		QString port = actionsPort->checkedAction()->text();
		if(port == "Custom")
		{
			port = customPort->text();
		}
		try
		{
			serialPort.setPort(port.toStdString());
			serialPort.setBaudrate(baudrate);
			serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
			serialPort.setTimeout(timeout);
			serialPort.open();
			timerSerial->start(0.01);
			edit->append("Connected to " +port + " with a baudrate of "+QString::number(baudrate));
		}
		catch(...)
		{
			edit->append("Failure to connect to: " + port);
			actionConnect->setEnabled(true);
			actionDisconnect->setEnabled(false);
			waypointsMissionSend->setEnabled(false);
		}
	}
}

void RobotControl::setRobotToPosition(double lat0, double lon0, double heading)
{
	counter_pointsTrace++;
	QString pointTraceName = "trace_" + QString::number(counter_pointsTrace);
	Q_EMIT drawCircleSignal(QGeoCoordinate(lat0,lon0),0.75,pointTraceName,"brown");
	pointsTrace.append(pointTraceName);
	if(hidingTrace)
	{
		QObject *circle = item->findChild<QObject*>(pointTraceName);
		if(circle)
		{	
			circle->setProperty("visible", false);
		}
	}

	cross->setProperty("visible", true);	
	setProj(lon0);
	double lat1,lat2,lat3;
	double lon1,lon2,lon3;
	double x0,x1,x2,x3;
	double y0,y1,y2,y3;

	latLonToXY(lat0,lon0,x0,y0);


	x1 = x0 + cos(heading+M_PI/2.)*2.;
	y1 = y0 + sin(heading+M_PI/2.)*2.;
	x2 = x0 + cos(heading)*6.;
	y2 = y0 + sin(heading)*6.;
	x3 = x0 + cos(heading-M_PI/2.)*2.;
	y3 = y0 + sin(heading-M_PI/2.)*2.;

	xyToLatLon(x1,y1,lat1,lon1);
	xyToLatLon(x2,y2,lat2,lon2);
	xyToLatLon(x3,y3,lat3,lon3);

	QMetaObject::invokeMethod(robot, "removeCoordinate",Q_ARG(int, 3));
	QMetaObject::invokeMethod(robot, "removeCoordinate",Q_ARG(int, 2));
	QMetaObject::invokeMethod(robot, "removeCoordinate",Q_ARG(int, 1));
	QMetaObject::invokeMethod(robot, "removeCoordinate",Q_ARG(int, 0));
	QMetaObject::invokeMethod(robot, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat1,lon1)));
	QMetaObject::invokeMethod(robot, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat2,lon2)));
	QMetaObject::invokeMethod(robot, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat3,lon3)));
	QMetaObject::invokeMethod(robot, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat1,lon1)));
}

void RobotControl::drawCOGVector(double lat0, double lon0, double cog_angle)
{
	double lat1;
	double lon1;
	double x0,x1;
	double y0,y1;

	latLonToXY(lat0,lon0,x0,y0);

	x1 = x0 + cos(cog_angle)*6.;
	y1 = y0 + sin(cog_angle)*6.;

	xyToLatLon(x1,y1,lat1,lon1);

	QMetaObject::invokeMethod(cogLine, "removeCoordinate",Q_ARG(int, 1));
	QMetaObject::invokeMethod(cogLine, "removeCoordinate",Q_ARG(int, 0));
	QMetaObject::invokeMethod(cogLine, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat0,lon0)));
	QMetaObject::invokeMethod(cogLine, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat1,lon1)));
}

void RobotControl::drawHeadingObjVector(double lat0, double lon0, double headingObj)
{
	double lat1;
	double lon1;
	double x0,x1;
	double y0,y1;


	headingObjLine->setProperty("visible", true);
	latLonToXY(lat0,lon0,x0,y0);

	x1 = x0 + cos(headingObj)*6.;
	y1 = y0 + sin(headingObj)*6.;

	xyToLatLon(x1,y1,lat1,lon1);

	QMetaObject::invokeMethod(headingObjLine, "removeCoordinate",Q_ARG(int, 1));
	QMetaObject::invokeMethod(headingObjLine, "removeCoordinate",Q_ARG(int, 0));
	QMetaObject::invokeMethod(headingObjLine, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat0,lon0)));
	QMetaObject::invokeMethod(headingObjLine, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat1,lon1)));
}


void RobotControl::hideCross()
{
	cross->setProperty("visible", false);	
}

void RobotControl::hideHeadingObj()
{
	headingObjLine->setProperty("visible", false);
}

void RobotControl::setCrossToPosition(double lat0, double lon0)
{
	double lat1,lat2,lat3,lat4;
	double lon1,lon2,lon3,lon4;
	double x0,x1,x2,x3,x4;
	double y0,y1,y2,y3,y4;

	latLonToXY(lat0,lon0,x0,y0);

	x1 = x0 + 7.;
	y1 = y0 + 7.;
	x2 = x0 - 7.;
	y2 = y0 - 7.;
	x3 = x0 - 7.;
	y3 = y0 + 7.;
	x4 = x0 + 7.;
	y4 = y0 - 7.;

	xyToLatLon(x1,y1,lat1,lon1);
	xyToLatLon(x2,y2,lat2,lon2);
	xyToLatLon(x3,y3,lat3,lon3);
	xyToLatLon(x4,y4,lat4,lon4);

	QMetaObject::invokeMethod(cross, "removeCoordinate",Q_ARG(int, 4));
	QMetaObject::invokeMethod(cross, "removeCoordinate",Q_ARG(int, 3));
	QMetaObject::invokeMethod(cross, "removeCoordinate",Q_ARG(int, 2));
	QMetaObject::invokeMethod(cross, "removeCoordinate",Q_ARG(int, 1));
	QMetaObject::invokeMethod(cross, "removeCoordinate",Q_ARG(int, 0));
	QMetaObject::invokeMethod(cross, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat1,lon1)));
	QMetaObject::invokeMethod(cross, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat2,lon2)));
	QMetaObject::invokeMethod(cross, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat0,lon0)));
	QMetaObject::invokeMethod(cross, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat3,lon3)));
	QMetaObject::invokeMethod(cross, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(lat4,lon4)));

}

void RobotControl::setInfos()
{
	if(posReceived)
	{
		lat->setText(QString::number(currentPos.lat,'f',8) + " deg");
		lon->setText(QString::number(currentPos.lon,'f',8) + " deg");
		alt->setText(QString::number(currentPos.alt,'f',2) + " m");
		error_xy->setText(QString::number(currentPos.error_xy,'f',2) + " m");
		error_alt->setText(QString::number(currentPos.error_h,'f',2) + " m");
	}
	if(attitudeReceived)
	{
		heading->setText(QString::number(currentAttitude.heading*180./M_PI,'f',1) + " deg  |  " + QString::number(currentAttitude.heading,'f',2) + " rad");
		roll->setText(QString::number(currentAttitude.roll*180./M_PI,'f',1) + " deg  |  " + QString::number(currentAttitude.roll,'f',2) + " rad");
		pitch->setText(QString::number(currentAttitude.pitch*180./M_PI,'f',1) + " deg  |  " + QString::number(currentAttitude.pitch,'f',2) + " rad");
	}
	if(headingObjReceived)
	{
		headingObj->setText(QString::number(currentHeadingObj.heading*180./M_PI,'f',1) + " deg  |  " + QString::number(currentHeadingObj.heading,'f',2) + " rad" );
	}
	if(overGroundReceived)
	{
		cog->setText(QString::number(currentOverGround.cog*180./M_PI,'f',1) + " (\u00b1 "+QString::number(currentOverGround.cog_acc*180./M_PI,'f',2)+ ") deg  |  " + QString::number(currentOverGround.cog,'f',2) + " rad");
		sog->setText(QString::number(currentOverGround.sog,'f',2) + " (\u00b1 "+QString::number(currentOverGround.sog_acc,'f',2)+") m/s  |  " + QString::number(currentOverGround.sog*3.6,'f',2) + " km/h");
	}
	if(posReceived and headingObjReceived)
	{
		drawHeadingObjVector(currentPos.lat,currentPos.lon,currentHeadingObj.heading);
	}
	if(posReceived and attitudeReceived)
	{
		setRobotToPosition(currentPos.lat,currentPos.lon,currentAttitude.heading);
	}
	if(posReceived and overGroundReceived)
	{
		drawCOGVector(currentPos.lat,currentPos.lon,currentOverGround.cog);
	}
	if(speedReceived)
	{
		speed->setValue(currentSpeed.speed*100.);
		speedReceived = false;
	}
	if(customReceived)
	{
		for(int i = 0; i < 4; i++)
		{
			if(currentCustom.isValid[i] == 0x01)
			{
				customLabels[i]->setText(QString::number(currentCustom.data[i],'f',2) + " " + customUnits[i]);
			}
			else
			{
				customLabels[i]->setText("");
			}
		}
	}

	if(modeReceived)
	{
		if(currentMode.mode == 1 or currentMode.mode == 2)
		{
			if(distToLineReceived)
			{
				distToLine->setText(QString::number(currentDistToLine.distToLine) + " m");
			}

			if(currentWpReceived)
			{
				setCrossToPosition(currentWp.lat,currentWp.lon);
				timeSinceStart->setText(QString::number(currentWp.timeMission,'f',1) +" s");

				if(overGroundReceived and currentWpReceived and currentOverGround.sog != 0)
				{
					distToWp->setText(QString::number(currentWp.dist,'f',2) + " m  |  " + QString::number(currentWp.dist/currentOverGround.sog,'f',2) +" s");
					distToEnd->setText(QString::number(currentWp.distTot,'f',2) + " m  |  " + QString::number(currentWp.distTot/currentOverGround.sog,'f',2) +" s");

				}
				else
				{
					distToWp->setText(QString::number(currentWp.dist,'f',2) + " m");
					distToEnd->setText(QString::number(currentWp.distTot,'f',2) + " m");
				}
			}


			if(currentMode.mode == 1)
			{
				mode->setText("<font color=\"red\">ARMED</font> - Waypoints following");
			}
			else
			{
				mode->setText("<font color=\"red\">ARMED</font> - Lines following");
			}
		

		}
		else
		{
			if(currentMode.mode == 3)
			{
				mode->setText("<font color=\"red\">ARMED</font> - Heading following");
			}
			else
			{
				mode->setText("<font color=\"green\">DISARMED</font>");
				headingObj->setText("");
			}
			distToLine->setText("");
			distToWp->setText("");
			distToEnd->setText("");

			hideCross();
			hideHeadingObj();
		}
	}
}

void RobotControl::unpackMessage(std::vector<uint8_t> data)
{
	QString messageName = "";
	bool parsed = false;
	if(messageType == 0x01)
	{
		parsed = p->unpackPosMsg(data,currentPos);
		messageName = "pos";
		posReceived = parsed or posReceived;
		actionCenterOnRobot->setEnabled(posReceived);

	}
	else if(messageType == 0x02)
	{
		parsed = p->unpackAttitudeMsg(data,currentAttitude);
		messageName = "attitude";
		attitudeReceived = parsed or attitudeReceived;
	}	
	else if(messageType == 0x03)
	{
		parsed = p->unpackOverGroundMsg(data,currentOverGround);
		messageName = "overground";
		overGroundReceived = parsed or overGroundReceived;
	}	
	else if(messageType == 0x04)
	{
		parsed = p->unpackMissionMsg(data,currentMission);
		messageName = "mission";
		missionReceived = parsed;
		if(parsed)
		{
			if(currentMission.override == 0x01)
			{
				edit->append("Receiving waypoints");
				listObjectivesReceive.clear();
				numberOfObjectivesToReceive = currentMission.totalObjectives;

			}
			for(int i = 0; i < currentMission.objectives.size(); i++)
			{
				listObjectivesReceive.push_back(currentMission.objectives[i]);
			}
			edit->append(QString::number(listObjectivesReceive.size())+" waypoints received");
			if(listObjectivesReceive.size() == numberOfObjectivesToReceive)
			{
				edit->append("All waypoints received");
				clearAllWaypoints();
				bool statusCheckbox = linesWaypointsMission->isChecked();
				settingWaypoints = true;
				for(int i = 0; i < numberOfObjectivesToReceive; i++)
				{
					linesWaypointsMission->setChecked(listObjectivesReceive[i].isLine);
					callbackWaypoint(listObjectivesReceive[i].lat,listObjectivesReceive[i].lon,true);	
				}
				linesWaypointsMission->setChecked(statusCheckbox);
				settingWaypoints = false;
			}

		}
	}
	else if(messageType == 0x05)
	{
		parsed = p->unpackSpeedMsg(data,currentSpeed);
		messageName = "speed";
		speedReceived = parsed;
	}
	else if(messageType == 0x06)
	{
		parsed = p->unpackOrderMsg(data,currentOrder);
		messageName = "order";
	}
	else if(messageType == 0x07)
	{
		parsed = p->unpackCurrentWpMsg(data,currentWp);
		messageName = "currentwp";
		currentWpReceived = parsed;
	}
	else if(messageType == 0x08)
	{
		parsed = p->unpackDistToLineMsg(data,currentDistToLine);
		messageName = "disttoline";
		distToLineReceived = parsed;
	}
	else if(messageType == 0x09)
	{
		parsed = p->unpackHeadingMsg(data,currentHeadingObj);
		messageName = "headingobj";
		headingObjReceived = parsed;
	}
	else if(messageType == 0x10)
	{
		parsed = p->unpackModeMsg(data,currentMode);
		messageName = "mode";
		modeReceived = parsed;
	}
	else if(messageType == 0x11)
	{
		parsed = p->unpackCustomMsg(data,currentCustom);
		messageName = "custom";
		customReceived = parsed;
	}
	if(parsed)
	{
		if(messageName == "order")
		{
			if(currentOrder.order == 0x05)
			{
				edit->append("Robot received waypoints");
			}
			else
			{
				edit->append("Unknown order received");	
			}
		}
		else
		{
			if(actionShowCorrectMessages->isChecked())
			{
				edit->append("Correct message " +messageName+ " received");
			}
			setInfos();
		}
		counter_messages++;

	}
	else if(messageName == "")
	{
		edit->append("Unknown message received");
	}
	else
	{
		edit->append("Bad message " +messageName+ " received");
	}
}

void RobotControl::serialRead()
{
	try
	{
		if(serialPort.isOpen() and serialPort.available())
		{
			std::string ch;
			int r = serialPort.read(ch,1);
			if(r == 1)
			{
				bytesReceivedPerSec++;
				buffer.push_back((uint8_t)ch[0]);
				int s = buffer.size();
				//edit->append(QString::number((int)(uint8_t)ch[0],16));
				if(bytesRemaining > 0)
				{
					bytesRemaining--;
					if(bytesRemaining == 0)
					{
						unpackMessage(buffer);
						buffer.clear();
						bytesRemaining = -1;
						messageType = 0x0;
					}
				}
				else if(s >= 6)
				{
					if(buffer[s-6] == 0xab and buffer[s-5] == 0xcd and buffer[s-4] == 0xef)
					{
						messageType = buffer[s-3];
						bytesRemaining =  buffer[s-2]+buffer[s-1]*256 - 6;
						buffer.erase(buffer.begin(),buffer.end()-6);
					}
				}
			}
		}
		timerSerial->start(0.01);
	}
	catch(...)
	{
		serialDisconnect();
	}
}

void RobotControl::serialDisconnect()
{
	timerSerial->stop();
	serialPort.close();
	edit->append("Disconnected from serial port");
	actionConnect->setEnabled(true);
	actionDisconnect->setEnabled(false);
	waypointsMissionSend->setEnabled(false);
}

void RobotControl::send(std::vector<uint8_t> data)
{
	try
	{
		if(serialPort.isOpen())
		{
			//edit->append(QString::number(data.size()));
			int written = serialPort.write(data);
			bytesSentPerSec += written;
			if(written != data.size())
			{
				edit->append("Unable to send data");
			}
			else
			{
				//edit->append("Message sent");
			}
		}
		else
		{
			edit->append("Serial port is closed.");
		}
	}
	catch(...)
	{
		serialDisconnect();
	}
}

void RobotControl::sendGetSpeed()
{
	std::vector<uint8_t> dataToSend = p->packOrderMsg(0x01);
	edit->append("Asking robot's speed");
	send(dataToSend);
}

void RobotControl::sendGetWaypoints()
{
	std::vector<uint8_t> dataToSend = p->packOrderMsg(0x02);
	edit->append("Asking robot's waypoints");
	send(dataToSend);
}

void RobotControl::sendRobotStart()
{
	std::vector<uint8_t> dataToSend = p->packOrderMsg(0x03);
	edit->append("Sending start");
	send(dataToSend);
}

void RobotControl::sendRobotHeadingStart()
{

	std::vector<uint8_t> dataToSend = p->packHeadingMsg(headingHeadingMission->value()*M_PI/180.);
	edit->append("Start following heading: " + QString::number(headingHeadingMission->value()) + " deg");
	send(dataToSend);
}

void RobotControl::sendRobotStop()
{
	std::vector<uint8_t> dataToSend = p->packOrderMsg(0x04);
	edit->append("Sending stop");
	send(dataToSend);
}

void RobotControl::sendSetSpeed()
{
	std::vector<uint8_t> dataToSend = p->packSpeedMsg((double)speedValue->intValue()/100.);
	edit->append("Setting speed to " + QString::number(speedValue->intValue()) + "%");
	send(dataToSend);
}

void RobotControl::sendObjectives()
{
	std::vector<objectiveWpOrLine> toSend;
	uint8_t override;
	if(objectivesToSendId == 0)
	{
		override = 0x01;
	}
	else
	{
		override = 0x00;
	}
	int s = std::min((int)objectivesToSend.size(),objectivesToSendId+20);
	int firstObjectiveToSendId = objectivesToSendId;
	while(objectivesToSendId < s)
	{
		toSend.push_back(objectivesToSend[objectivesToSendId]);
		objectivesToSendId++;
	}
	std::vector<uint8_t> dataToSend = p->packMissionMsg(toSend,objectivesToSend.size(),override);
	edit->append("Sending waypoints " + QString::number(firstObjectiveToSendId) + " to "+ QString::number(s));
	send(dataToSend);
	if(objectivesToSendId == objectivesToSend.size())
	{
		objectivesToSendId = 0;
		objectivesToSend.clear();
		edit->append("All waypoints sent");
		timerSendObjectives->stop();
	}
	else
	{
		timerSendObjectives->start(200);
	}
}

void RobotControl::sendWaypoints()
{
	if(waypoints.size() >= 2)
	{
		std::vector<objectiveWpOrLine> v;
		for(int i = 0; i < waypoints.size(); i++)
		{
			objectiveWpOrLine o;
			o.lat = waypoints[i].lat; 
			o.lon = waypoints[i].lon;
			o.isLine = waypoints[i].lineMode;
			v.push_back(o);
		}
		objectivesToSend = v;
		objectivesToSendId = 0;
		timerSendObjectives->start(1);

	}
	else
	{
		edit->append("Not enought waypoints...");
	}

}

void RobotControl::setMovingWaypointMode()
{
	
	if(listOfWaypoints->currentRow() >= 0)
	{
		int wpId = -1;
		for(int i = 0; i < waypoints.size(); i++)
		{
			if(waypoints[i].elem == listOfWaypoints->currentItem())
			{
				wpId = i;
				break;
			}
		}
		if(wpId)
		{
			//QString txt = listOfWaypoints->currentItem()->text().split(" ")[0];
			startSelectWaypoints->setEnabled(false);
			stopSelectWaypoints->setEnabled(true);
			clearWaypoints->setEnabled(false);
			moveWaypoint->setEnabled(false);
			waypointsMissionSend->setEnabled(false);
			waypointsMissionLoad->setEnabled(false);
			actionSave->setEnabled(false);
			actionOpen->setEnabled(false);
			edit->append("Ready to move waypoint");	
			movingWaypoint = true;
			waypointToMoveId = wpId;
		}
		else
		{
			edit->append("Nothing to move");			
		}
	}
	else
	{
		edit->append("Nothing to move");	
	}
}

bool RobotControl::findWaypointByName(QString name, waypoint &wp)
{
	for(int i = 0; i < waypoints.size(); i++)
	{
		if(waypoints[i].name == name)
		{
			wp = waypoints[i];
			return true;
		}
	}
	return false;
}

void RobotControl::dynamicReconfigureWaypoints()
{
	for(int i = 0; i < waypoints.size(); i++)
	{
			QObject *circle = item->findChild<QObject*>(waypoints[i].name);
			if(circle)
			{	
		    	circle->setProperty("radius", waypointsSizeValue);
				QQmlProperty property_lat(circle, "center.latitude");
				QQmlProperty property_lon(circle, "center.longitude");
				property_lat.write(waypoints[i].lat);
				property_lon.write(waypoints[i].lon);
			}
	}
}

void RobotControl::dynamicReconfigureLines()
{
	for(int i = 0; i < lines.size(); i++)
	{
			QObject *line = item->findChild<QObject*>(lines[i].name);
			waypoint wp1,wp2;

			if(line and findWaypointByName(lines[i].wp1Name,wp1) and findWaypointByName(lines[i].wp2Name,wp2))
			{	
				QMetaObject::invokeMethod(line, "removeCoordinate",Q_ARG(int, 1));
				QMetaObject::invokeMethod(line, "removeCoordinate",Q_ARG(int, 0));
				QMetaObject::invokeMethod(line, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(wp1.lat,wp1.lon)));
				QMetaObject::invokeMethod(line, "addCoordinate", Q_ARG(QGeoCoordinate, QGeoCoordinate(wp2.lat,wp2.lon)));
			}
	}
}

void RobotControl::changeWaypointsSize(int i)
{
	waypointsSizeValue = i;
	dynamicReconfigureWaypoints();
}

void RobotControl::setWaypoints()
{
	edit->append("Start setting waypoints");
	settingWaypoints = true;
	startSelectWaypoints->setEnabled(false);
	stopSelectWaypoints->setEnabled(true);
	clearWaypoints->setEnabled(false);
	moveWaypoint->setEnabled(false);
	waypointsMissionSend->setEnabled(false);
	waypointsMissionLoad->setEnabled(false);
	actionSave->setEnabled(false);
	actionOpen->setEnabled(false);

}

void RobotControl::stopSettingWaypoints()
{
	settingWaypoints = false;
	startSelectWaypoints->setEnabled(true);
	stopSelectWaypoints->setEnabled(false);
	clearWaypoints->setEnabled(true);
	moveWaypoint->setEnabled(true);
	waypointsMissionSend->setEnabled(true);
	waypointsMissionLoad->setEnabled(true);
	actionSave->setEnabled(true);
	actionOpen->setEnabled(true);
	edit->append("Stop Setting waypoints");
}

RobotControl::~RobotControl()
{
	delete settings;
	delete p;
	delete timerSerial;
	delete robot;
	delete cogLine;
	delete headingObjLine;
	delete cross;
	delete statusBar;
	delete timerStatus;
	delete timerListSerial;
	delete menus;
	delete menuFile;
	delete menuSerial;
	delete menuPort;
	delete customPort;
	delete menuBaudrate;
	delete menuConfig;
	delete waypointsSize;
	delete centralWidget;
	delete timerSendObjectives;
}
