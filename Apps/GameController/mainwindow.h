#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QFileDialog>
#include <QMessageBox>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bimap.hpp>
#include <boost/asio.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <mmsystem.h>

// DirectInput
#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#pragma comment (lib,"dinput8.lib")
#pragma comment (lib,"dxguid.lib")

// LSL API
#include <lsl_cpp.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT
	
public:
	explicit MainWindow(QWidget *parent, const std::string &config_file);
	~MainWindow();
	
private slots:
	// config file dialog ops (from main menu)
	void load_config_dialog();
	void save_config_dialog();

	// configuration
	void hertzLineEdit_editingFinished();

	// start the GameController connection
	void link();

	// close event (potentially disabled)
	void closeEvent(QCloseEvent *ev);
private:
	static BOOL CALLBACK controller_enum_callback(const DIDEVICEINSTANCE* pdidInstance, VOID *pWindow);
	static BOOL CALLBACK object_enum_callback(const DIDEVICEOBJECTINSTANCE* pdidoi, VOID *pContext);

	BOOL CALLBACK on_controller(const DIDEVICEINSTANCE* pdidInstance);

	// background data reader thread
	void read_thread(std::string name);

	// timer callback
	static void CALLBACK s_timer_callback(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2);

	// raw config file IO
	void load_config(const std::string &filename);
	void save_config(const std::string &filename);
	int iEventsPerSecond_;

	// game-controller reading support
	boost::posix_time::ptime ptSystemStart_;
	double dLocaltimeStart_;
	lsl::stream_info *infoButtons_;
	lsl::stream_outlet *outletButtons_;
	lsl::stream_info *infoAxes_;
	lsl::stream_outlet *outletAxes_;
	bool waspressed_[128];
	boost::posix_time::ptime t_start_;
	boost::int64_t t_;
	std::filebuf logFile_;
	std::ostream *logStream_;
	void gamecontroller_start(std::string name);
	void gamecontroller_frame();
	void gamecontroller_stop();

	bool stop_;											// whether we're trying to stop the reader thread
	LPDIRECTINPUT8 pDI;									// pointer to DirectInput instance
	LPDIRECTINPUTDEVICE8 pController;					// pointer to controller instance
	boost::shared_ptr<boost::thread> reader_thread_;	// our reader thread
	MMRESULT timer_;									// our timer handle

	boost::bimap<int,std::wstring> indexToInstance_;	// map between index in device selection list and instance GUID string

	Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
