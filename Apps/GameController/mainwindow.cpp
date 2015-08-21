#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }

MainWindow::MainWindow(QWidget *parent, const std::string &config_file) :
QMainWindow(parent), ui(new Ui::MainWindow), pDI(NULL), pController(NULL)
{
	try {
		// No timer yet.
		timer_ = NULL;

		// Get our time reference, i.e. so that LSL local-clock times can
		// be converted to UTC date/time.
		ptSystemStart_ = boost::posix_time::microsec_clock::universal_time();
		dLocaltimeStart_ = lsl::local_clock();

		ui->setupUi(this);

		HRESULT hr;
		// Init DirectInput
		if (FAILED(hr=DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8, (VOID**)&pDI,NULL)))
			throw std::runtime_error("Could not create DirectInput instance. Please make sure that you have DirectX 9.0 installed.");

		// Enumerate game controllers and add them to the UI
		ui->deviceSelector->clear();
		if (FAILED(hr=pDI->EnumDevices(DI8DEVCLASS_GAMECTRL, controller_enum_callback, this, DIEDFL_ATTACHEDONLY)))
			throw std::runtime_error("Could not enumerate game controllers. Please make sure that you have a controller plugged in and that you DirectX is properly installed.");
		if (indexToInstance_.empty())
			throw std::runtime_error("No game controllers are plugged in.");

		// set up validation
		ui->hertzLineEdit->setValidator(new QIntValidator(1, 100, ui->hertzLineEdit));

		// parse startup config file
		load_config(config_file);

		// make GUI connections
		QObject::connect(ui->actionQuit, SIGNAL(triggered()), this, SLOT(close()));
		QObject::connect(ui->linkButton, SIGNAL(clicked()), this, SLOT(link()));
		QObject::connect(ui->actionLoad_Configuration, SIGNAL(triggered()), this, SLOT(load_config_dialog()));
		QObject::connect(ui->actionSave_Configuration, SIGNAL(triggered()), this, SLOT(save_config_dialog()));
		QObject::connect(ui->hertzLineEdit, SIGNAL(editingFinished()), this, SLOT(hertzLineEdit_editingFinished()));
	} catch(std::exception &e) {
		QMessageBox::critical(this,"Error",e.what(),QMessageBox::Ok);
		throw;
	}
}

// enumerates controller features and sets ranges for the axes
BOOL CALLBACK MainWindow::object_enum_callback(const DIDEVICEOBJECTINSTANCE* pdidoi, VOID *pWindow) {
	if(pdidoi->dwType & DIDFT_AXIS) {
		DIPROPRANGE diprg; 
		diprg.diph.dwSize       = sizeof(DIPROPRANGE); 
		diprg.diph.dwHeaderSize = sizeof(DIPROPHEADER); 
		diprg.diph.dwHow        = DIPH_BYID; 
		diprg.diph.dwObj        = pdidoi->dwType;
		diprg.lMin              = -1000; 
		diprg.lMax              = +1000; 
		// Set the range for the axis
		((MainWindow*)pWindow)->pController->SetProperty(DIPROP_RANGE,&diprg.diph);
	}
	return DIENUM_CONTINUE;
}

// enumerates controllers and populates GUI combobox with them
BOOL CALLBACK MainWindow::controller_enum_callback(const DIDEVICEINSTANCE* pdidInstance, VOID *pWindow) {
	return ((MainWindow*)pWindow)->on_controller(pdidInstance);
}

BOOL CALLBACK MainWindow::on_controller(const DIDEVICEINSTANCE* pdidInstance) {
	wchar_t *bstrGuid = NULL;
	StringFromCLSID(pdidInstance->guidInstance,&bstrGuid);
	indexToInstance_.insert(boost::bimap<int,std::wstring>::value_type(ui->deviceSelector->count(),std::wstring(bstrGuid)));
	char instance_name[4096]; wcstombs(instance_name,pdidInstance->tszInstanceName,sizeof(instance_name));
	ui->deviceSelector->addItem(instance_name);
	::CoTaskMemFree(bstrGuid);
	return DIENUM_CONTINUE;
}


void MainWindow::load_config_dialog() {
	QString sel = QFileDialog::getOpenFileName(this,"Load Configuration File","","Configuration Files (*.cfg)");
	if (!sel.isEmpty())
		load_config(sel.toStdString());
}

void MainWindow::save_config_dialog() {
	QString sel = QFileDialog::getSaveFileName(this,"Save Configuration File","","Configuration Files (*.cfg)");
	if (!sel.isEmpty())
		save_config(sel.toStdString());
}

void MainWindow::closeEvent(QCloseEvent *ev) {
	if (reader_thread_)
		ev->ignore();
}

void MainWindow::load_config(const std::string &filename) {
	using boost::property_tree::wptree;
	wptree pt;

	// parse file
	try {
		read_xml(filename, pt);
	} catch(std::exception &e) {
		QMessageBox::information(this,"Error",(std::string("Cannot read config file: ")+= e.what()).c_str(),QMessageBox::Ok);
		return;
	}

	// get config values
	try {
		// pre-select the device in the UI
		std::wstring deviceguid = pt.get<std::wstring>(L"settings.deviceguid",L"");
		if (!deviceguid.empty()) {
			if (!indexToInstance_.right.count(deviceguid)) {
				QMessageBox::information(this,"Error","The previously configured device was not found. Is it plugged in?",QMessageBox::Ok);
			} else
				ui->deviceSelector->setCurrentIndex(indexToInstance_.right.at(deviceguid));
		}

		// get value for events-per-second
		iEventsPerSecond_ = pt.get(L"settings.eventsPerSecond", 1);
		ui->hertzLineEdit->setText(boost::lexical_cast<std::string>(iEventsPerSecond_).c_str());

		// get value for use-timer-event
		bool bUseTimer = pt.get(L"settings.useTimerEvent", false);
		ui->timerCheckBox->setChecked(bUseTimer);

		// get value for write-log
		bool bWriteLog = pt.get(L"settings.writeLog", false);
		ui->logCheckBox->setChecked(bWriteLog);

		// determine if "link" button can be enabled
		this->hertzLineEdit_editingFinished();
	} catch(std::exception &) {
		QMessageBox::information(this,"Error in Config File","Could not read out config parameters.",QMessageBox::Ok);
		return;
	}
}

void MainWindow::save_config(const std::string &filename) {
	using boost::property_tree::wptree;
	wptree pt;

	// transfer UI content into property tree
	try {
		pt.put(L"settings.deviceguid",indexToInstance_.left.at(ui->deviceSelector->currentIndex()));
		if (iEventsPerSecond_ != 1 /* default */)
			pt.put(L"settings.eventsPerSecond", iEventsPerSecond_);
		if (ui->timerCheckBox->isChecked() != false /* default */)
			pt.put(L"settings.useTimerEvent", ui->timerCheckBox->isChecked());
		if (ui->logCheckBox->isChecked() != false /* default */)
			pt.put(L"settings.writeLog", ui->logCheckBox->isChecked());
	} catch(std::exception &e) {
		QMessageBox::critical(this,"Error",(std::string("Could not prepare settings for saving: ")+=e.what()).c_str(),QMessageBox::Ok);
	}

	// write to disk
	try {
		write_xml(filename, pt);
	} catch(std::exception &e) {
		QMessageBox::critical(this,"Error",(std::string("Could not write to config file: ")+=e.what()).c_str(),QMessageBox::Ok);
	}
}

void MainWindow::hertzLineEdit_editingFinished()
{
	// Get the update rate.
	QByteArray oString = ui->hertzLineEdit->text().toAscii();
	const char *pszString = oString.constData();
	sscanf(pszString, "%d", &iEventsPerSecond_);

	// Input is valid, so allow linking.
	ui->linkButton->setEnabled(true);
}

// start/stop the GameController connection
void MainWindow::link() {
	HRESULT hr;
	if (reader_thread_) {
		// === perform unlink action ===
		try {
			stop_ = true;
			reader_thread_->interrupt();
			reader_thread_->join();
			reader_thread_.reset();
			// unacquire, release and delete everything...
			pController->Unacquire();
			SAFE_RELEASE(pController);
			SAFE_DELETE(pController);
		} catch(std::exception &e) {
			QMessageBox::critical(this,"Error",(std::string("Could not stop the background processing: ")+=e.what()).c_str(),QMessageBox::Ok);
			return;
		}

		// Allow configuration changes again.
		ui->deviceSelector->setEnabled(true);
		ui->hertzLineEdit->setEnabled(true);
		ui->timerCheckBox->setEnabled(true);
		ui->logCheckBox->setEnabled(true);

		// indicate that we are now successfully unlinked
		ui->linkButton->setText("Link");
	} else if (timer_ != NULL) {
		// Stop the timer.
		::timeKillEvent(timer_);
		timer_ = NULL;

		// Stop reading from the game-controller.
		gamecontroller_stop();

		// Allow configuration changes again.
		ui->deviceSelector->setEnabled(true);
		ui->hertzLineEdit->setEnabled(true);
		ui->timerCheckBox->setEnabled(true);
		ui->logCheckBox->setEnabled(true);

		// indicate that we are now successfully unlinked
		ui->linkButton->setText("Link");
	} else {
		// === perform link action ===
		try {
			// get the UI parameters (selected GUID)
			std::wstring guidstr = indexToInstance_.left.at(ui->deviceSelector->currentIndex());
			std::string name = ui->deviceSelector->itemText(ui->deviceSelector->currentIndex()).toStdString();
			GUID guid;
			if (FAILED(hr=CLSIDFromString((LPOLESTR)guidstr.c_str(),&guid)))
				throw std::runtime_error("Did not find the selected device. Is it plugged in?");

			// Obtain an interface to the selected joystick
			if (FAILED(hr=pDI->CreateDevice(guid,&pController,NULL)))
				throw std::runtime_error("Could not instantiate the selected device. Is it plugged in?");

			// and select the data format
			if (FAILED(hr=pController->SetDataFormat(&c_dfDIJoystick2)))
				throw std::runtime_error("Could not select data format for the controller. The controller might not be fully compatible with DirectInput.");

			// set cooperative level
			if (FAILED(hr=pController->SetCooperativeLevel(winId(), DISCL_BACKGROUND | DISCL_NONEXCLUSIVE)))
				throw std::runtime_error("Could not set cooperative level for the device. There might be another application running that demands exclusive access to the device.");

			// try to enumerate the device features; we'll do this to set the ranges for the axes
			if (FAILED(hr=pController->EnumObjects(object_enum_callback,this,DIDFT_ALL)))
				throw std::runtime_error("Could not enumerate device properties. The device might not be fully compatible with DirectInput.");

			// acquire it
			if (FAILED(hr=pController->Acquire()))
				throw std::runtime_error("Could not acquire access to the selected controller. Is another program using it in exclusive mode?");

			// start reading
			if (ui->timerCheckBox->isChecked())
			{
				// Set up to read from the game-controller.
				gamecontroller_start(name);

				// Set up the timer.
				timer_ = ::timeSetEvent(1000 / iEventsPerSecond_, 1, &s_timer_callback,
					(DWORD_PTR)this, TIME_PERIODIC | TIME_KILL_SYNCHRONOUS);
			}
			else
			{
				stop_ = false;
				reader_thread_.reset(new boost::thread(&MainWindow::read_thread,this,name));
			}
		}
		catch(std::exception &e) {
			QMessageBox::critical(this,"Error",(std::string("Could not initialize the GameController interface: ")+=e.what()).c_str(),QMessageBox::Ok);
			return;
		}

		// Disallow configuration changes while linked.
		ui->deviceSelector->setEnabled(false);
		ui->hertzLineEdit->setEnabled(false);
		ui->timerCheckBox->setEnabled(false);
		ui->logCheckBox->setEnabled(false);

		// done, all successful
		ui->linkButton->setText("Unlink");
	}
}

void MainWindow::gamecontroller_start(std::string name)
{
	// create streaminfo and outlet for the button events
	infoButtons_ = new lsl::stream_info(name + "Buttons","GameControllerMarkers",1,lsl::IRREGULAR_RATE,lsl::cf_string,name + "_Buttons_" + boost::asio::ip::host_name());
	outletButtons_ = new lsl::stream_outlet(*infoButtons_);

	// create streaminfo and outlet for the axes
	infoAxes_ = new lsl::stream_info(name + "Axes","GameControllerPosition",36,60,lsl::cf_float32,name + "_Axes_" + boost::asio::ip::host_name());
	// append some meta-data...
	lsl::xml_element channels = infoAxes_->desc().append_child("channels");
	channels.append_child("channel")
		.append_child_value("label","X")
		.append_child_value("type","PositionX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","Y")
		.append_child_value("type","PositionY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","Z")
		.append_child_value("type","PositionZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","RX")
		.append_child_value("type","RotationX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","RY")
		.append_child_value("type","RotationY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","RZ")
		.append_child_value("type","RotationZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","U")
		.append_child_value("type","PositionU")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","V")
		.append_child_value("type","PositionV")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","PovN")
		.append_child_value("type","Position")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","PovE")
		.append_child_value("type","Position")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","PovS")
		.append_child_value("type","Position")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","PovW")
		.append_child_value("type","Position")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VX")
		.append_child_value("type","VelocityX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VY")
		.append_child_value("type","VelocityY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VZ")
		.append_child_value("type","VelocityZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VRX")
		.append_child_value("type","AngularVelocityX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VRY")
		.append_child_value("type","AngularVelocityY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VRZ")
		.append_child_value("type","AngularVelocityZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VU")
		.append_child_value("type","VelocityU")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","VV")
		.append_child_value("type","VelocityV")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","AX")
		.append_child_value("type","AccelerationX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","AY")
		.append_child_value("type","AccelerationY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","AZ")
		.append_child_value("type","AccelerationZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","ARX")
		.append_child_value("type","AngularAccelerationX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","ARY")
		.append_child_value("type","AngularAccelerationY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","ARZ")
		.append_child_value("type","AngularAccelerationZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","AU")
		.append_child_value("type","AccelerationU")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","AV")
		.append_child_value("type","AccelerationV")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FX")
		.append_child_value("type","ForceX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FY")
		.append_child_value("type","ForceY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FZ")
		.append_child_value("type","ForceZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FRX")
		.append_child_value("type","TorqueX")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FRY")
		.append_child_value("type","TorqueY")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FRZ")
		.append_child_value("type","TorqueZ")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FU")
		.append_child_value("type","ForceU")
		.append_child_value("unit","normalized_signed");
	channels.append_child("channel")
		.append_child_value("label","FV")
		.append_child_value("type","ForceV")
		.append_child_value("unit","normalized_signed");
	infoAxes_->desc().append_child("acquisition")
		.append_child_value("model",name.c_str());
	outletAxes_ = new lsl::stream_outlet(*infoAxes_);

	for (int i = 0; i < _countof(waspressed_); ++i)
		waspressed_[i] = false;
	t_start_ = boost::posix_time::microsec_clock::local_time();
	t_=0;

	// Open a log file with the current date/time.
	if (ui->logCheckBox->isChecked())
	{
		boost::posix_time::ptime ptNow = boost::posix_time::microsec_clock::universal_time();
		std::locale loc(std::cout.getloc(),
			new boost::posix_time::time_facet("%Y%m%d_%H%M%S"));
		std::basic_stringstream<char> ss;
		ss.imbue(loc);
		ss << "log_" << ptNow << ".csv";
		logFile_.open(ss.str().c_str(), std::ios::out);
		logStream_ = new std::ostream(&logFile_);
		(*logStream_) << "Timestamp,X,Y,Z" << std::endl;
	}
}

void MainWindow::gamecontroller_frame()
{
	HRESULT hr;
	DIJOYSTATE2 js;	

	// poll the device
	if (FAILED(hr=pController->Poll()))
		while (!stop_ && (hr = pController->Acquire()) == DIERR_INPUTLOST) ;

	// obtain its input state
	if (FAILED(hr=pController->GetDeviceState(sizeof(DIJOYSTATE2),&js)))
		QMessageBox::critical(this,"Error","Cannot obtain device state.",QMessageBox::Ok);

	double now = lsl::local_clock();

	// construct the axes sample and send it off
	float sample[36] = {js.lX,js.lY,js.lZ,js.lRx,js.lRy,js.lRz,js.rglSlider[0],js.rglSlider[1],
		js.rgdwPOV[0],js.rgdwPOV[1],js.rgdwPOV[2],js.rgdwPOV[3],js.lVX,js.lVY,js.lVZ,js.lVRx,js.lVRy,js.lVRz,
		js.rglVSlider[0],js.rglVSlider[1],js.lAX,js.lAY,js.lAZ,js.lARx,js.lARy,js.lARz,js.rglASlider[0],
		js.rglASlider[1],js.lFX,js.lFY,js.lFZ,js.lFRx,js.lFRy,js.lFRz,js.rglFSlider[0],js.rglFSlider[1]};
	// scale the numbers
	for (int k=0;k<36;k++)
		sample[k] /= 1000.0;
	outletAxes_->push_sample(sample,now);

	if (ui->logCheckBox->isChecked())
	{
		// Convert the seconds-since-epoch value to a system time.
		boost::posix_time::ptime ptNow = ptSystemStart_
			+ boost::posix_time::microseconds((now - dLocaltimeStart_) * 1000000.0);
		std::string strTime = boost::gregorian::to_iso_extended_string_type<char>(ptNow.date())
			+ " " + boost::posix_time::to_simple_string_type<char>(ptNow.time_of_day()) + "Z";

		// Log this game-controller sample.
		(*logStream_) << strTime << "," << js.lX << "," << js.lY << "," << js.lZ << std::endl;
		/* char aszMsg[256];
		sprintf (aszMsg, "%s,%ld,%ld,%ld\n", strTime.c_str(),
			js.lX, js.lY, js.lZ);
		::OutputDebugStringA(aszMsg); */
	}

	// generate the button-event samples...
	for (int i=0; i<128; i++) {
		if ((js.rgbButtons[i]&0x80)) {
			if (!waspressed_[i]) {
				waspressed_[i] = true;
				std::string text("Button" + boost::lexical_cast<std::string>(i) + " pressed");
				outletButtons_->push_sample(&text,now);
			}
		} else {
			if (waspressed_[i]) {
				waspressed_[i] = false;
				std::string text("Button" + boost::lexical_cast<std::string>(i) + " released");
				outletButtons_->push_sample(&text,now);
			}
		}
	}		
}

void MainWindow::gamecontroller_stop()
{
	if (ui->logCheckBox->isChecked())
	{
		logStream_->flush();
		logFile_.close();
		SAFE_DELETE(logStream_);
	}

	SAFE_DELETE(outletAxes_);
	SAFE_DELETE(infoAxes_);
	SAFE_DELETE(outletButtons_);
	SAFE_DELETE(infoButtons_);
}

// background data reader thread
void MainWindow::read_thread(std::string name) {
	gamecontroller_start(name);

	// enter transmission loop
	while (!stop_) {
		gamecontroller_frame();
		boost::this_thread::sleep(t_start_ + boost::posix_time::millisec((++t_)*(1000 / iEventsPerSecond_)));
	}

	gamecontroller_stop();
}

// timer callback
void CALLBACK MainWindow::s_timer_callback(UINT /* uTimerID */, UINT /* uMsg */, DWORD_PTR dwUser,
	DWORD_PTR /* dw1 */, DWORD_PTR /* dw2 */)
{
	MainWindow *pThis = (MainWindow *)dwUser;
	pThis->gamecontroller_frame();
}

MainWindow::~MainWindow() {
	delete ui;
	SAFE_RELEASE(pDI);
	SAFE_DELETE(pDI);
}

