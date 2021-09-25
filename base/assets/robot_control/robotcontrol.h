#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <QtWidgets>
#include <QtCore>
#include <QQuickWidget>
#include <QQuickItem>
#include <QQuickView>
#include <QQmlContext>
#include <QGeoCoordinate>
#include <QQmlProperty>
#include "serial/serial.h"
#include "protocol.h"
#include <proj.h>



struct waypoint
{
	double lat;
	double lon;
	QListWidgetItem *elem;
	QString name;
	bool lineMode;
	QString lineName;
};

struct line
{
	QString wp1Name;
	QString wp2Name;
	QString name;
};

class RobotControl : public QMainWindow
{
	Q_OBJECT

	public:
	    RobotControl();
	    ~RobotControl();

	Q_SIGNALS:
    	void drawCircleSignal(const QGeoCoordinate &coordinates, const double &size, const QString &name, const QString &color);
    	void drawLineSignal(const QGeoCoordinate &coordinates1, const QGeoCoordinate &coordinates2,const int &size, const QString &name, const QString &color);


	private slots:
		void callbackWaypoint(double lat, double lon, bool isLeft);
		void callbackDoubleClick(double lat, double lon, double zoomLevel, bool forward);
		void setWaypoints();
		void stopSettingWaypoints();
		void clearAllWaypoints();
		void changeWaypointsSize(int i);
		void setMovingWaypointMode();
		void serialConnect();
		void serialRead();
		void serialDisconnect();
		void centerOnRobot();
		void sendWaypoints();
		void sendSetSpeed();
		void sendGetSpeed();
		void sendRobotStart();
		void sendRobotStop();
		void sendGetWaypoints();
		void sendRobotHeadingStart();
		void saveCurrentMission();
		void loadMission();
		void clearTrace();
		void hideTraceChangedStatus(bool i);
		void setStatusMessage();
		void addSerialPortsToMenu();
		void sendObjectives();
		void setWaypointSurvey();
		void generateSurvey();

	private:
		void dynamicReconfigureWaypoints();
		void dynamicReconfigureLines();
		bool findWaypointByName(QString name, waypoint &wp);
		void unpackMessage(std::vector<uint8_t> data);
		void setInfos();
		void setRobotToPosition(double lat0, double lon0, double heading);
		void drawCOGVector(double lat0, double lon0, double cog_angle);
		void drawHeadingObjVector(double lat0, double lon0, double headingObj);
		void setCrossToPosition(double lat0, double lon0);
		void latLonToXY(double lat_, double lon_, double &x_, double &y_);
		void xyToLatLon(double x_, double y_,double &lat_, double &lon_);
		void setProj(double lon = -4.519289017);
		void hideCross();
		void hideHeadingObj();
		void send(std::vector<uint8_t> data);


		Protocol *p;
		QTimer *timerSerial;
		serial::Serial serialPort;
		std::vector<uint8_t> buffer;
		int bytesRemaining;
		uint8_t messageType;


		QList<waypoint> waypoints;
		QList<line> lines;
		QStringList pointsTrace;
		bool settingWaypoints;
		bool movingWaypoint;
		int counter_waypoints;
		int counter_lines;
		int counter_pointsTrace;

		int waypointsSizeValue;
		QString waypointsColorValue;
		int waypointToMoveId;

		bool hidingTrace;


		posMsg currentPos;
		attitudeMsg currentAttitude;
		missionMsg currentMission;
		speedMsg currentSpeed;
		currentWpMsg currentWp;
		distToLineMsg currentDistToLine;
		modeMsg currentMode;
		overGroundMsg currentOverGround;
		headingMsg currentHeadingObj;
		customMsg currentCustom;
		orderMsg currentOrder;

		bool posReceived;
		bool attitudeReceived;
		bool speedReceived;
		bool missionReceived;
		bool currentWpReceived;
		bool distToLineReceived;
		bool modeReceived;
		bool overGroundReceived;
		bool headingObjReceived;
		bool customReceived;

		PJ_CONTEXT *C;
		PJ *P;
		PJ* P_for_GIS;
		PJ_COORD a, b;

		QObject *robot;
		QObject *cogLine;
		QObject *headingObjLine;
		QObject *cross;

		QTimer *timerSendObjectives;
		std::vector<objectiveWpOrLine> objectivesToSend;
		int objectivesToSendId;

		std::vector<objectiveWpOrLine> listObjectivesReceive;
		int numberOfObjectivesToReceive;

		QStatusBar *statusBar;
		QTimer *timerStatus;

		int bytesReceivedPerSec;
		int bytesSentPerSec;
		int counter_messages;

		QTimer *timerListSerial;
		QSettings *settings;

		double wpSurveyLat;
		double wpSurveyLon;

		QMenuBar *menus;

			QMenu *menuFile;
			QAction *actionOpen;
			QAction *actionSave;
			QAction *actionExit;

			QMenu *menuSerial;
				QMenu *menuPort;
				QActionGroup *actionsPort;
						QAction *actionCustom;
						QLineEdit *customPort;
						QWidgetAction *actionCustomPort;

				QMenu *menuBaudrate;
					QActionGroup *actionsBaudrate;

						QAction *b115200;
						QAction *b57600;
						QAction *b38400;
						QAction *b19200;
						QAction *b9600;
						QAction *b921600;

				QAction *actionConnect;
				QAction *actionDisconnect;


			QMenu *menuConfig;
				QMenu *menuWaypointsSize; 
					QWidgetAction *actionWaypointsSize;
					QSpinBox *waypointsSize;
				QAction *actionClearLog;
				QAction *actionShowCorrectMessages;
				QAction *actionClearTrace;
				QAction *actionShowTrace;
			QAction *actionCenterOnRobot;

		QWidget *centralWidget;
			QGridLayout *centralLayout;
					QQuickWidget *view;
					QObject *item;

				QTextEdit *edit;

				QVBoxLayout *cmdLayout;
					QGroupBox *mission;
							QVBoxLayout *missionLayout;
							QTabWidget *tabMission;

								QWidget *waypointsMission;
									QFormLayout *waypointsMissionLayout;
									QCheckBox *linesWaypointsMission;
									QListWidget *listOfWaypoints;
									QHBoxLayout *buttonsWaypointsMission1;
										QPushButton *startSelectWaypoints;
										QPushButton *stopSelectWaypoints;
										QPushButton *clearWaypoints;
										QPushButton *moveWaypoint;
									QHBoxLayout *buttonsWaypointsMission2;
										QPushButton *waypointsMissionSend;
										QPushButton *waypointsMissionLoad;

								QWidget *robotMission;
									QVBoxLayout *robotMissionLayout;
										QGroupBox *speedBox;
											QVBoxLayout *speedLayout;
												QHBoxLayout *speedSetLayout;
												QSlider *speed;
												QLCDNumber *speedValue;
												QHBoxLayout *speedButtonLayout;
													QPushButton *setSpeed;
													QPushButton *getSpeed;
										QGroupBox *missionWp;
											QHBoxLayout *missionWpLayout;
													QPushButton *startRobotMission;
										QGroupBox *missionHeading;
											QFormLayout *headingMissionLayout;
												QDoubleSpinBox *headingHeadingMission;
												QHBoxLayout *robotMissionButtonsLayout2;
													QPushButton *headingMissionStart;
										QPushButton *stopRobot;
								QWidget *other;
									QFormLayout *otherLayout;
										QLabel *cog;
										QLabel *sog;
										QLabel *distToLine;
										QLabel *distToWp;
										QLabel *distToEnd;
										QLabel *mode;
										QLabel *headingObj;
										QLabel *timeSinceStart;
										QList<QLabel *> customLabels;
										QList<QString> customUnits;
								QWidget *survey;
									QFormLayout *surveyLayout;
									QPushButton *loadLastWp;
									QLabel *lastWp;
									QDoubleSpinBox *surveyLength;
									QDoubleSpinBox *surveyWidth;
									QDoubleSpinBox *surveyNumberOfRails;
									QDoubleSpinBox *surveyYaw;
									QPushButton *surveyGenerate;




					QGroupBox *status;
						QVBoxLayout *statusLayout;
							QGroupBox *pos;
								QFormLayout *posLayout;
									QLabel *lat;
									QLabel *lon;
									QLabel *alt;
									QLabel *error_xy;
									QLabel *error_alt;
							QGroupBox *attitude;
								QFormLayout *attitudeLayout;
									QLabel *heading;
									QLabel *pitch;
									QLabel *roll;

};

#endif
