#include "dual_foc.h"
#include "ui_dual_foc.h"

#include "zeng_data_receive.h"

#include "QMessageBox"
#include "qwt_compass.h"
#include "qwt_compass_rose.h"
#include "qwt_dial_needle.h"

#include <iostream>
#include <fstream>
#include <windows.h>

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

using namespace std;

#define PI (3.141592653f)
#define dt	(0.03)
#define FRAME_TAIL		(0x4844)

extern float rad_deg_sw;

dual_foc::dual_foc(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::dual_foc)
{
	ui->setupUi(this);
	System_Init();
}

dual_foc::~dual_foc()
{
	delete ui;
}

const char *Curve_name[DIS_LINE_NUM]={
	"gyro_x",
	"gyro_z",
	"accel_x",
	"gyro_y",
	"accel_z",
	"accel_y",
	"roll",
	"pitch",
	"yaw",
	"mx",
	"my",
	"mz",
	"height",
	"acc",
	"mag",
	"est_roll",
	"est_pitch"
};

const QColor Curve_color[DIS_LINE_NUM]={
	Qt::red,
	Qt::white,
	Qt::green,
	Qt::blue,
	Qt::lightGray,
	Qt::yellow,
	Qt::magenta,
	Qt::darkCyan,
	Qt::darkYellow,
	Qt::gray,
	Qt::darkMagenta,
	Qt::cyan,
	Qt::color0,
	Qt::magenta,
	Qt::darkGreen,
	Qt::white,
	Qt::red
};

void dual_foc::Zeng_Curve_init(LINE_DATA *Curve, uint16_t thumb)
{
	//static uint16_t thumb = 0;
	Curve->Curve = new QwtPlotCurve();
	Curve->Curve->setTitle(Curve_name[thumb]);
	Curve->Curve->setPen(Curve_color[thumb], 1);
	for(int i=0; i<DIS_DATA_LENTH; i++)
	{
		Curve->y_data.append(10*sin(dt*i+2*PI*thumb/DIS_LINE_NUM));
	}
	Curve->Curve->setSamples(x_data, Curve->y_data);
	Curve->Curve->attach(ui->Dis_plot);
	//thumb++;
}

void dual_foc::Zeng_Curve_clean(LINE_DATA *Curve)
{
	Curve->y_data.clear();
	for(int i=0; i<DIS_DATA_LENTH; i++)
	{
		Curve->y_data.append(0);
	}
	//Curve->Curve->setSamples(x_data, Curve->y_data);
}

void dual_foc::config_file_init(void)
{
	config_file = fopen("config.bin", "rb");
	fread((void*)&upper_config, 1, sizeof(upper_config), config_file);
}

void dual_foc::System_Init(void)
{
    QwtText title;
    QFont font;
	update_timer = new QTimer();

//	qDebug()<<"sizeof(IMU_PORT_SET_TYPE):"<<sizeof(IMU_PORT_SET_TYPE)<<endl;

	config_file_init();

    mag_cal_flag = false;
    mag_cnt = 0;
    for(uint8_t i=0; i<3; i++)
    {
        for(uint8_t j=0; j<2; j++)
        {
            mag_scale[i][j] = 0;
        }
    }
/******************************* Init compass ********************************/
	ui->Compass->setLineWidth(5);
	ui->Compass->setFrameShadow(QwtCompass::Raised);

	QwtCompassScaleDraw *scaleDraw = new QwtCompassScaleDraw;
	scaleDraw->enableComponent(QwtAbstractScaleDraw::Ticks, true);
	scaleDraw->enableComponent(QwtAbstractScaleDraw::Labels, true);
	scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
	scaleDraw->setTickLength(QwtScaleDiv::MinorTick, 1);
	scaleDraw->setTickLength(QwtScaleDiv::MediumTick, 3);
	scaleDraw->setTickLength(QwtScaleDiv::MajorTick, 5);
	ui->Compass->setScaleDraw(scaleDraw);

	ui->Compass->setNeedle(new QwtCompassMagnetNeedle(QwtCompassMagnetNeedle::TriangleStyle, Qt::black, Qt::red));
	ui->Compass->setValue(90);
/******************************* Init compass ********************************/

	Rad_Degree_sw = new QButtonGroup(this);
	Rad_Degree_sw->addButton(ui->rad_sw, 1);
	Rad_Degree_sw->addButton(ui->deg_sw,2);
	ui->rad_sw->setChecked(true);


//	ui->tabWidget->widget(0)->setBackgroundRole(Qt::blue);
    ui->Dis_plot->setTitle("IMU data");
	ui->Dis_plot->setCanvasBackground(Qt::black);
    ui->Dis_plot->insertLegend(new QwtLegend(), QwtPlot::RightLegend);

    font.setFamily("Times New Roman");
    font.setPointSize(10);
    font.setBold(false);
    title.setFont(font);
    title.setText("x-y plane");
    ui->mag_xy->setTitle(title);
    ui->mag_xy->setAxisTitle(QwtPlot::xBottom, "x");
    ui->mag_xy->setAxisTitle(QwtPlot::yLeft, "y");

	title.setText("y-z plane");
    ui->mag_xz->setTitle(title);
	ui->mag_xz->setAxisTitle(QwtPlot::xBottom, "y");
    ui->mag_xz->setAxisTitle(QwtPlot::yLeft, "z");

	title.setText("z-x plane");
    ui->mag_yz->setTitle(title);
	ui->mag_yz->setAxisTitle(QwtPlot::xBottom, "z");
	ui->mag_yz->setAxisTitle(QwtPlot::yLeft, "x");

    font.setPointSize(6);
    ui->mag_xy->setAxisFont(QwtPlot::xBottom, font);
    ui->mag_xy->setAxisFont(QwtPlot::yLeft, font);
    ui->mag_xz->setAxisFont(QwtPlot::xBottom, font);
    ui->mag_xz->setAxisFont(QwtPlot::yLeft, font);
    ui->mag_yz->setAxisFont(QwtPlot::xBottom, font);
    ui->mag_yz->setAxisFont(QwtPlot::yLeft, font);

	ui->tabWidget->setCurrentIndex(0);

	for(int i=0; i<DIS_DATA_LENTH; i++)
	{
		x_data.append(dt*i);
    }

	for(int i=0; i<DIS_LINE_NUM; i++)
	{
		Zeng_Curve_init(&Line[i], i);
	}

//    for(int i=0; i<628; i++)
//    {
//        mag_data[0].append(sin(0.01*i));
//        mag_data[1].append(cos(0.01*i));
//        mag_data[2].append(0.01*i);
//    }
    mag_curve[0] = new QwtPlotCurve();
    mag_curve[0]->setPen(Qt::blue);
    mag_curve[0]->attach(ui->mag_xy);
    StdCir_curve[0] = new QwtPlotCurve();
    StdCir_curve[0]->setPen(Qt::red);
    StdCir_curve[0]->attach(ui->mag_xy);

    mag_curve[1] = new QwtPlotCurve();
    mag_curve[1]->setPen(Qt::blue);
    mag_curve[1]->attach(ui->mag_xz);
    StdCir_curve[1] = new QwtPlotCurve();
    StdCir_curve[1]->setPen(Qt::red);
    StdCir_curve[1]->attach(ui->mag_xz);

    mag_curve[2] = new QwtPlotCurve();
    mag_curve[2]->setPen(Qt::blue);
    mag_curve[2]->attach(ui->mag_yz);
    StdCir_curve[2] = new QwtPlotCurve();
    StdCir_curve[2]->setPen(Qt::red);
    StdCir_curve[2]->attach(ui->mag_yz);

	//ui->Dis_plot->detachItems(0);
	//Zeng_Curve_init(&Line[1]);

	connect(update_timer, SIGNAL(timeout()), this, SLOT(Update_timer_handler()));
	update_timer->stop();

	foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
	{
		QSerialPort serial;
		serial.setPort(info);

		if(serial.open(QIODevice::ReadWrite))
		{
			serial.close();
			ui->Port_name->addItem(serial.portName());
		}
		qDebug() << serial.portName() << endl;
	}

	if((upper_config.port_name_index>0)&&(upper_config.port_name_index<30))
	{
		ui->Port_name->setCurrentIndex(upper_config.port_name_index);
	}

	ui->baudrate->setCurrentIndex(upper_config.baudrate_index);
	ui->baudrate_set->setCurrentIndex(upper_config.baudrate_index);
	ui->frequency_set->setCurrentIndex(8);
	Zeng_data_receive_init();

	ui->voltage->display(0);

	Zeng_SW_button_state(false);

	for(int i=0;i<DIS_LINE_NUM;i++)
	{
		line_flag[i]=0;
	}

	//default config
	if(upper_config.gx)
	{
		ui->gx_select->setChecked(true);
		line_flag[0] = 2;
	}else{
		ui->gx_select->setChecked(false);
		line_flag[0] = 0;
	}

	if(upper_config.gz)
	{
		ui->gz_select->setChecked(true);
		line_flag[1] = 2;
	}else{
		ui->gz_select->setChecked(false);
		line_flag[1] = 0;
	}

	if(upper_config.ax)
	{
		ui->ax_select->setChecked(true);
		line_flag[2] = 2;
	}else{
		ui->ax_select->setChecked(false);
		line_flag[2] = 0;
	}

	if(upper_config.gy)
	{
		ui->gy_select->setChecked(true);
		line_flag[3] = 2;
	}else{
		ui->gy_select->setChecked(false);
		line_flag[3] = 0;
	}

	if(upper_config.az)
	{
		ui->az_select->setChecked(true);
		line_flag[4] = 2;
	}else{
		ui->az_select->setChecked(false);
		line_flag[4] = 0;
	}

	if(upper_config.ay)
	{
		ui->ay_select->setChecked(true);
		line_flag[5] = 2;
	}else{
		ui->ay_select->setChecked(false);
		line_flag[5] = 0;
	}

	if(upper_config.roll_sel)
	{
		ui->roll_select->setChecked(true);
		line_flag[6] = 2;
	}else{
		ui->roll_select->setChecked(false);
		line_flag[6] = 0;
	}

	if(upper_config.pitch_sel)
	{
		ui->pitch_selsect->setChecked(true);
		line_flag[7] = 2;
	}else{
		ui->pitch_selsect->setChecked(false);
		line_flag[7] = 0;
	}

	if(upper_config.yaw_sel)
	{
		ui->yaw_selsect->setChecked(true);
		line_flag[8] = 2;
	}else{
		ui->yaw_selsect->setChecked(false);
		line_flag[8] = 0;
	}

	if(upper_config.mx_sel)
	{
		ui->mx_select->setChecked(true);
		line_flag[9] = 2;
	}else{
		ui->mx_select->setChecked(false);
		line_flag[9] = 0;
	}

	if(upper_config.my_sel)
	{
		ui->my_select->setChecked(true);
		line_flag[10] = 2;
	}else{
		ui->my_select->setChecked(false);
		line_flag[10] = 0;
	}

	if(upper_config.mz_sel)
	{
		ui->mz_select->setChecked(true);
		line_flag[11] = 2;
	}else{
		ui->mz_select->setChecked(false);
		line_flag[11] = 0;
	}

	if(upper_config.h_sel)
	{
		ui->h_select->setChecked(true);
		line_flag[12] = 2;
	}else{
		ui->h_select->setChecked(false);
		line_flag[12] = 0;
	}

	if(upper_config.acc_sel)
	{
		ui->acc_select->setChecked(true);
		line_flag[13] = 2;
	}else{
		ui->acc_select->setChecked(false);
		line_flag[13] = 0;
	}

	if(upper_config.mag_sel)
	{
		ui->mag_select->setChecked(true);
		line_flag[14] = 2;
	}else{
		ui->mag_select->setChecked(false);
		line_flag[14] = 0;
	}

	if(upper_config.est_roll_sel)
	{
		ui->est_roll_select->setChecked(true);
		line_flag[15] = 2;
	}else{
		ui->est_roll_select->setChecked(false);
		line_flag[15] = 0;
	}

	if(upper_config.est_pitch_sel)
	{
		ui->est_pitch_select->setChecked(true);
		line_flag[16] = 2;
	}else{
		ui->est_pitch_select->setChecked(false);
		line_flag[16] = 0;
	}

	if(upper_config.unit_rad == 1)
	{
		rad_deg_sw = 1;
		Rad_Degree_sw->button(1)->setChecked(true);
		qDebug()<<"rad"<<endl;
	}else{
		rad_deg_sw = 57.2957795f;
		Rad_Degree_sw->button(2)->setChecked(true);
		qDebug()<<"angle" <<endl;
	}
	qDebug()<<"unit value:" <<(short)upper_config.unit_rad<<endl;
}

void dual_foc::Zeng_curve_update(LINE_DATA *curve, double data)
{
	//删除第一个数据
	curve->y_data.erase(curve->y_data.begin(), curve->y_data.begin()+1);
	//将新数据补在最后
	curve->y_data.append(data);
	curve->Curve->setSamples(x_data, curve->y_data);
	curve->Curve->attach(ui->Dis_plot);
}

void dual_foc::Zeng_Curve_display(void)
{
	for(int i=0; i<DIS_LINE_NUM; i++)
	{
		if(line_flag[i]>0)
		{
			Line[i].Curve->setSamples(x_data, Line[i].y_data);
			Line[i].Curve->attach(ui->Dis_plot);
			ui->Dis_plot->replot();
		}else{
		}
	}


}

void dual_foc::Update_timer_handler(void)
{
	//static uint32_t i = 0;
	char voltage_dis_str[5];
	float update_line_data[DIS_LINE_NUM];
//	Vector3f temp_acc, linar_acc;
//	static SEND_DATA_TYPE last_dataType = SYSTEM_DATA;
//	static bool datatype_sw = false;
	if(dis_data.update_flag)
	{
		dis_data.update_flag = false;

        if(2 == ui->tabWidget->currentIndex())
        {
//            qDebug()<<"tab 2"<<endl;
            ui->HD_IMU_3D_display->update();
        }

        if(mag_cal_flag)
        {
            if(mag_scale[0][0] < dis_data.mag[0])
            {
                mag_scale[0][0] = dis_data.mag[0];
                ui->mag_Xmax->setText(QString::number(mag_scale[0][0]));
            }
            if(mag_scale[0][1] > dis_data.mag[0])
            {
                mag_scale[0][1] = dis_data.mag[0];
                ui->mag_Xmin->setText(QString::number(mag_scale[0][1]));
            }

            if(mag_scale[1][0] < dis_data.mag[1])
            {
                mag_scale[1][0] = dis_data.mag[1];
                ui->mag_Ymax->setText(QString::number(mag_scale[1][0]));
            }
            if(mag_scale[1][1] > dis_data.mag[1])
            {
                mag_scale[1][1] = dis_data.mag[1];
                ui->mag_Ymin->setText(QString::number(mag_scale[1][1]));
            }

            if(mag_scale[2][0] < dis_data.mag[2])
            {
                mag_scale[2][0] = dis_data.mag[2];
                ui->mag_Zmax->setText(QString::number(mag_scale[2][0]));
            }
            if(mag_scale[2][1] > dis_data.mag[2])
            {
                mag_scale[2][1] = dis_data.mag[2];
                ui->mag_Zmin->setText(QString::number(mag_scale[2][1]));
            }
            mag_data[0].append(dis_data.mag[0]);
            mag_data[1].append(dis_data.mag[1]);
            mag_data[2].append(dis_data.mag[2]);

            mag_curve[0]->setSamples(mag_data[0], mag_data[1]);
//            mag_curve[0]->attach(ui->mag_xy);
            ui->mag_xy->replot();

			mag_curve[1]->setSamples(mag_data[1], mag_data[2]);
//            mag_curve[1]->attach(ui->mag_xz);
            ui->mag_xz->replot();

			mag_curve[2]->setSamples(mag_data[2], mag_data[0]);
//            mag_curve[2]->attach(ui->mag_yz);
            ui->mag_yz->replot();

            mag_offset[0] = (mag_scale[0][0]+mag_scale[0][1])/2.0f;
            mag_offset[1] = (mag_scale[1][0]+mag_scale[1][1])/2.0f;
            mag_offset[2] = (mag_scale[2][0]+mag_scale[2][1])/2.0f;
            ui->mag_offset_x->setText(QString::number(mag_offset[0]));
            ui->mag_offset_y->setText(QString::number(mag_offset[1]));
            ui->mag_offset_z->setText(QString::number(mag_offset[2]));
        }

		if(dis_data.data_type&(0x01<<EULER_FLOAT))
		{
			ui->gyro_x_dis->setText(QString::number(dis_data.euler[0], 'f', 5));
			ui->gyro_y_dis->setText(QString::number(dis_data.euler[1], 'f', 5));
			ui->gyro_z_dis->setText(QString::number(dis_data.euler[2], 'f', 5));
		}

		if(dis_data.data_type&(0x01<<ACCEL_FLOAT))
		{
			ui->accel_x_dis->setText(QString::number(dis_data.acc[0], 'f', 5));
			ui->accel_y_dis->setText(QString::number(dis_data.acc[1], 'f', 5));
			ui->accel_z_dis->setText(QString::number(dis_data.acc[2], 'f', 5));
		}

		if(dis_data.data_type&(0x01<<EST_EULER))
		{
			ui->est_rol_dis->setText(QString::number(dis_data.est_euler[0], 'f', 5));
			ui->est_pit_dis->setText(QString::number(dis_data.est_euler[1], 'f', 5));
		}


		update_line_data[0] = dis_data.gyro[0];
		update_line_data[3] = dis_data.gyro[1];
		update_line_data[1] = dis_data.gyro[2];


		update_line_data[2] = dis_data.acc[0];
		update_line_data[5] = dis_data.acc[1];
		update_line_data[4] = dis_data.acc[2];

		update_line_data[6] = dis_data.euler[0];
		update_line_data[7] = dis_data.euler[1];
		update_line_data[8] = dis_data.euler[2];

		update_line_data[9] = dis_data.mag[0];
		update_line_data[10] = dis_data.mag[1];
		update_line_data[11] = dis_data.mag[2];

		update_line_data[13] = dis_data.acceleration;
		update_line_data[14] = dis_data.magnetic;

		update_line_data[LINE_EST_ROLL] = dis_data.est_euler[0];
		update_line_data[LINE_EST_PIT] = dis_data.est_euler[1];

		if(dis_data.euler[2]>0)
		{
			ui->Compass->setValue(-dis_data.euler[2]*57.2957795f/rad_deg_sw+360);
		}else{
			ui->Compass->setValue(-dis_data.euler[2]*57.2957795f/rad_deg_sw);
		}

		for(int i=0; i<DIS_LINE_NUM; i++)
		{
			if(line_flag[i]>0)
			{
				Zeng_curve_update(&Line[i], update_line_data[i]);
			}else{
				//Zeng_curve_update(&Line[i], 0);
			}
		}

		ui->Dis_plot->replot();
        sprintf(voltage_dis_str, "%.1f", dis_data.voltage);
		ui->voltage->display(QString(voltage_dis_str));


		ui->system_time_dis->setText(QString::number(dis_data.system_time));
#ifdef SAVE_CAL_DATA
		ui->M0_target_current->setText(QString::number(/*0.1*ui->verticalSlider_vel_left->value()*/real_angle));
#else
#endif

//		last_dataType = dis_data.data_type;
	}


	//i++;
}

void dual_foc::FOC_display_message(QString message)
{
	ui->terminal_dis->insertPlainText(message);
	ui->terminal_dis->moveCursor(QTextCursor::End);
}

void dual_foc::Zeng_SW_button_state(bool state)
{
	bool inv_state = (state==true?false:true);
	ui->gyro_offset_button->setEnabled(state);
	ui->acc_offset_botton->setEnabled(state);
	ui->reset_button->setEnabled(state);

	ui->set_button->setEnabled(state);
    ui->start_mag_cal_button->setEnabled(state);
    ui->write_mag_cal_button->setEnabled(state);

	ui->baudrate->setEnabled(inv_state);
	ui->Port_name->setEnabled(inv_state);
}

void dual_foc::on_uart_open_button_clicked()
{
	int err = 0;
	static bool first_open = true;

	if("Open" == ui->uart_open_button->text())
	{
		QByteArray ba = ui->Port_name->currentText().toLatin1();
		err = Zeng_data_ready_task(ba.data()/*ui->Port_name->currentIndex()*/, ui->baudrate->currentText().toInt());
		if(0 == err)
		{
			FOC_display_message("Open "+ui->Port_name->currentText()\
											  + " buadrate:"\
											  + ui->baudrate->currentText()+" successful!\n");
			ui->uart_open_button->setText("Close");
			//ui->Port_name->setEnabled(false);
			//ui->baudrate->setEnabled(false);
			Zeng_SW_button_state(true);
            update_timer->start(3);

			if(first_open)
			{
				//Zeng_Curve_clean
				for(int i=0; i<DIS_LINE_NUM; i++)
				{
					Zeng_Curve_clean(&Line[i]);
				}
				first_open = false;
			}

			for(int i=0; i<DIS_LINE_NUM; i++)
			{
				if(line_flag[i] == 0)
				{
					ui->Dis_plot->detachItems(Line[i].Curve->Rtti_PlotItem, false);
				}
			}

		}else{
			FOC_display_message("open "+ui->Port_name->currentText()\
											  + " buadrate:"\
											  + ui->baudrate->currentText()+" fail!\n");
		}
	}else{
		update_timer->stop();
		err = Zeng_data_suspend_task();
		if(0 == err)
		{
			FOC_display_message("Close "+ui->Port_name->currentText()\
											  + " buadrate:"\
											  + ui->baudrate->currentText()+" successful!\n");
			ui->uart_open_button->setText("Open");
			Zeng_SW_button_state(false);
		}else{
			FOC_display_message("Close "+ui->Port_name->currentText()\
											  + " buadrate:"\
											  + ui->baudrate->currentText()+" fail!\n");
		}

	}
}

void dual_foc::Zeng_uart_command_header_init(uint8_t type)
{
	uart_command.header.magic[0] = Header[0];
	uart_command.header.magic[1] = Header[1];
	uart_command.header.common_type = type;
	uart_command.header.data_len = sizeof(uart_command)-2;
}

void dual_foc::config_file_save(void)
{
	config_file = fopen("config.bin", "wb");
	fwrite((void*)&upper_config, 1, sizeof(upper_config), config_file);
	fclose(config_file);
}

void dual_foc::on_Port_name_currentIndexChanged(int index)
{
	upper_config.port_name_index = index;
	config_file_save();
}

void dual_foc::on_roll_select_stateChanged(int arg1)
{
	uint8_t thumb = 6;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.roll_sel = false;
		//upper_config.angle_sel = false;
	}else{
		upper_config.roll_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_pitch_selsect_stateChanged(int arg1)
{
	uint8_t thumb = 7;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.pitch_sel = false;
	}else{
		upper_config.pitch_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_yaw_selsect_stateChanged(int arg1)
{
	uint8_t thumb = 8;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.yaw_sel = false;
	}else{
		upper_config.yaw_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

extern bool save_flag;
ofstream save_data;
void dual_foc::on_save_data_button_clicked()
{
	if("save data" == ui->save_data_button->text())
	{
		save_data.open("data//data.csv");
		save_data<<"ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,temp"<<endl;
		ui->save_data_button->setText("close save");
		save_flag = true;
	}else{
		save_data.close();
		ui->save_data_button->setText("save data");
		save_flag = false;
	}
}

void dual_foc::on_mz_select_stateChanged(int arg1)
{
	uint8_t thumb = 11;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.mz_sel = false;
	}else{
		upper_config.mz_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_mx_select_stateChanged(int arg1)
{
	uint8_t thumb = 9;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.mx_sel = false;
	}else{
		upper_config.mx_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_my_select_stateChanged(int arg1)
{
	uint8_t thumb = 10;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.my_sel = false;
	}else{
		upper_config.my_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_acc_select_stateChanged(int arg1)
{
	uint8_t thumb = 13;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.acc_sel = false;
	}else{
		upper_config.acc_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_mag_select_stateChanged(int arg1)
{
	uint8_t thumb = 14;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.mag_sel = false;
	}else{
		upper_config.mag_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_baudrate_currentIndexChanged(int index)
{
	upper_config.baudrate_index = index;
	config_file_save();
}

static const uint16_t data_output_frequency[10] = {1000, 200, 100, 40, 20, 10, 8, 4, 2, 1};
void dual_foc::on_set_button_clicked()
{
    IMU_PORT_SET_TYPE data;
    data.set.output_types = 0x00;
	if(Zeng_is_usart_open())
	{
        data.header.magic[0] = Header[0];
        data.header.magic[1] = Header[1];
        data.header.common_type = COMMAND_SET_PORT_CONFIG;
        data.header.data_len = sizeof(data)-4;

        data.set.baudrate = ui->baudrate_set->currentText().toInt();
        data.set.frequency = data_output_frequency[ui->frequency_set->currentIndex()];
        data.set.Usart = true;

		if(ui->gyro_check->checkState())
		{
            data.set.output_types |= (0x01<<GYRO_FLOAT);
		}
		if(ui->accel_check->checkState())
		{
            data.set.output_types |= (0x01<<ACCEL_FLOAT);
		}
		if(ui->mag_check->checkState())
		{
            data.set.output_types |= (0x01<<MAG_FLAOT);
		}
		if(ui->euler_check->checkState())
		{
            data.set.output_types |= (0x01<<EULER_FLOAT);
		}
		if(ui->quaternion_check->checkState())
		{
            data.set.output_types |= (0x01<<QUATERN_FLOAT);
		}
		if(ui->transformation_check->checkState()){
			data.set.output_types |= (0x01<<ROTATION_MATRIX);
		}
		if(ui->linar_acc_check->checkState()){
			data.set.output_types |= (0x01<<LINAR_ACC);
		}
		if(ui->est_euler_check->checkState()){
			data.set.output_types |= (0x01<<EST_EULER);
		}

        data.set_magic = FRAME_TAIL;

        data.CheckSum = Zeng_cal_CheckSum_uint16((uint16_t*)&data.header.common_type, data.header.data_len/2);
        if(0 == Zeng_uart_write_data(sizeof(data), (char*)&data))
        {
            FOC_display_message("Write port config successful!\n");

            update_timer->stop();
            Zeng_data_suspend_task();

            Zeng_SW_button_state(false);
            ui->baudrate->setCurrentIndex(ui->baudrate_set->currentIndex());
            ui->uart_open_button->setText("Open");


        }else{
            FOC_display_message("Write port config commond fail!\n");
        }

	}else{
		FOC_display_message("Open UART port first!\n");
	}
}

void dual_foc::on_h_select_stateChanged(int arg1)
{
//    Q_UNUSED(arg1);
	uint8_t thumb = 12;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.h_sel = false;
	}else{
		upper_config.h_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_start_mag_cal_button_clicked()
{
    if("Start" == ui->start_mag_cal_button->text())
    {
        ui->start_mag_cal_button->setText("Stop");

        ui->mag_radius_x->setText(QString::number(0));
        ui->mag_radius_y->setText(QString::number(0));
        ui->mag_radius_z->setText(QString::number(0));
        mag_cal_flag = true;
    }else{
        QVector<double> x_, y_;

        float R = (mag_scale[0][0]-mag_scale[0][1])/2.0f;
        StdCir_curve[0]->setPen(Qt::red);
        for(uint16_t i=0; i<314; i++)
        {
            x_.append(R*sin(0.02*i));
            y_.append(R*cos(0.02*i));
        }
        StdCir_curve[0]->setSamples(x_,y_);
        ui->mag_xy->replot();
        x_.clear();
        y_.clear();

        R = (mag_scale[1][0]-mag_scale[1][1])/2.0f;
        StdCir_curve[1]->setPen(Qt::red);
        for(uint16_t i=0; i<314; i++)
        {
            x_.append(R*sin(0.02*i));
            y_.append(R*cos(0.02*i));
        }
        StdCir_curve[1]->setSamples(x_, y_);
        ui->mag_xz->replot();
        x_.clear();
        y_.clear();

        R = (mag_scale[2][0]-mag_scale[2][1])/2.0f;
        StdCir_curve[2]->setPen(Qt::red);
        for(uint16_t i=0; i<314; i++)
        {
            x_.append(R*sin(0.02*i));
            y_.append(R*cos(0.02*i));
        }
        StdCir_curve[2]->setSamples(x_, y_);
        ui->mag_yz->replot();
        x_.clear();
        y_.clear();


        ui->mag_radius_x->setText(QString::number((mag_scale[0][0]-mag_scale[0][1])/2.0f));
        ui->mag_radius_y->setText(QString::number((mag_scale[1][0]-mag_scale[1][1])/2.0f));
        ui->mag_radius_z->setText(QString::number((mag_scale[2][0]-mag_scale[2][1])/2.0f));

        mag_data[0].clear();
        mag_data[1].clear();
        mag_data[2].clear();
        for(uint8_t i=0; i<3; i++)
        {
            for(uint8_t j=0; j<2; j++)
            {
                mag_scale[i][j] = 0;
            }
        }

        ui->start_mag_cal_button->setText("Start");
        mag_cal_flag = false;
    }
}

void dual_foc::on_write_mag_cal_button_clicked()
{
    IMU_MAG_OFFSET_TYPE data;
    if(Zeng_is_usart_open())
    {
        data.header.magic[0] = Header[0];
        data.header.magic[1] = Header[1];
        data.header.common_type = COMMAND_SET_MAG_CAL;
        data.header.data_len = sizeof(data)-4;

        data.offset[0] = mag_offset[0];
        data.offset[1] = mag_offset[1];
        data.offset[2] = mag_offset[2];


        data.magic = FRAME_TAIL;
        data.CheckSum = Zeng_cal_CheckSum_uint16((uint16_t*)&data.header.common_type, data.header.data_len/2);
        if(0 == Zeng_uart_write_data(sizeof(data), (char*)&data))
        {
            FOC_display_message("Write mag offset config successful!\n");

        }else{
            FOC_display_message("Write mag offset config commond fail!\n");
        }

    }else{
        FOC_display_message("Open UART port first!\n");
    }
}

void dual_foc::on_rad_sw_toggled(bool checked)
{
	if(checked)
	{
		rad_deg_sw = 1;
		upper_config.unit_rad = 1;
	}else{
		rad_deg_sw = 57.2957795f;
		upper_config.unit_rad = -5;
	}
	config_file_save();
//	qDebug()<<"rad deg sw value:"<<rad_deg_sw<<endl;
}

void dual_foc::on_gz_select_stateChanged(int arg1)
{
	//qDebug()<<"argument:"<<arg1<<endl;
	line_flag[1] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[1].Curve->Rtti_PlotItem, false);
		upper_config.gz = false;
	}else{
		upper_config.gz = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_gx_select_stateChanged(int arg1)
{
	line_flag[0] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[0].Curve->Rtti_PlotItem, false);
		upper_config.gx = false;
	}else{
		upper_config.gx = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_gy_select_stateChanged(int arg1)
{
	line_flag[3] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[3].Curve->Rtti_PlotItem, false);
		upper_config.gy = false;
	}else{
		upper_config.gy = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_ax_select_stateChanged(int arg1)
{
	line_flag[2] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[2].Curve->Rtti_PlotItem, false);
		upper_config.ax = false;
	}else{
		upper_config.ax = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_ay_select_stateChanged(int arg1)
{
	line_flag[5] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[5].Curve->Rtti_PlotItem, false);
		upper_config.ay = false;
	}else{
		upper_config.ay = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_az_select_stateChanged(int arg1)
{
	line_flag[4] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[4].Curve->Rtti_PlotItem, false);
		upper_config.az = false;
	}else{
		upper_config.az = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_gyro_offset_button_clicked()
{
	if(Zeng_is_usart_open())
		{
			int click = QMessageBox::warning(this, "警告", "确保IMU静止！", QMessageBox::Yes, QMessageBox::No);
			if(QMessageBox::Yes == click)
			{
				Zeng_uart_command_header_init(COMMAND_CAL);
				uart_command.motor_status[0] = DETECT_DCCAL;
				uart_command.motor_status[1] = STOP;
				uart_command.speed[0] = 250;//50*5
				uart_command.speed[1] = 0;
				uart_command.current[0] = 0;
				uart_command.current[1] = 0;

				if(0 == Zeng_uart_write_data(sizeof(uart_command), (char*)&uart_command))
				{
					FOC_display_message("UART Write gyro offset commond!\n");
				}else{
					FOC_display_message("UART Write gyro offset commond fail!\n");
				}
			}

		}else{
			FOC_display_message("Open UART port first!\n");
		}
}

void dual_foc::on_acc_offset_botton_clicked()
{
	//uart_command uart_command;
		if(Zeng_is_usart_open())
		{
			int click = QMessageBox::warning(this, "警告", "确保IMU静止并水平放置！", QMessageBox::Yes, QMessageBox::No);
			if(QMessageBox::Yes == click)
			{
				Zeng_uart_command_header_init(COMMAND_CAL);
				uart_command.motor_status[0] = DETECT_ENCODE;
				uart_command.motor_status[1] = STOP;
				uart_command.speed[0] = -150;//-30*5
				uart_command.speed[1] = 0;
				uart_command.current[0] = 0;
				uart_command.current[1] = 0;

				if(0 == Zeng_uart_write_data(sizeof(uart_command), (char*)&uart_command))
				{
					FOC_display_message("UART Write accel offset commond!\n");
				}else{
					FOC_display_message("UART Write accel offset commond fail!\n");
				}
			}
		}else{
			FOC_display_message("Open UART port first!\n");
		}
}


void dual_foc::on_est_roll_select_stateChanged(int arg1)
{
	uint8_t thumb = 15;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.est_roll_sel = false;
	}else{
		upper_config.est_roll_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}

void dual_foc::on_est_pitch_select_stateChanged(int arg1)
{
	uint8_t thumb = 16;
	line_flag[thumb] = arg1;
	if(arg1 == 0)
	{
		ui->Dis_plot->detachItems(Line[thumb].Curve->Rtti_PlotItem, false);
		upper_config.est_pitch_sel = false;
	}else{
		upper_config.est_pitch_sel = true;
	}
	config_file_save();
	if("Open" == ui->uart_open_button->text())
	{
		Zeng_Curve_display();
	}
}
