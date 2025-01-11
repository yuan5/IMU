#ifndef DUAL_FOC_H
#define DUAL_FOC_H

#include <QMainWindow>

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_plot_grid.h>
#include <QTimer>
#include <stdbool.h>
#include <fstream>
#include <QButtonGroup>
#include <Eigen\Eigen>

#include "zeng_data_receive.h"

#define UART	(0)
#define CAN		(1)

typedef enum LINE_NUM_NAME_{
	LINE_GX = 0x00,
	LINE_GZ,
	LINE_AX,
	LINE_GY,
	LINE_AZ,
	LINE_AY,
	LINE_ROLL,
	LINE_PITCH,
	LINE_YAW,
	LINE_MX,
	LINE_MY,
	LINE_MZ,
	LINE_HEIGHT,
	LINE_ACC,
	LINE_MAG,
	LINE_EST_ROLL,
	LINE_EST_PIT,
	DIS_LINE_NUM
}LINE_NUM_NAME;

using namespace std;

namespace Ui {
class dual_foc;
}

#define DIS_DATA_LENTH  (300)
//#define DIS_LINE_NUM	(17)

typedef struct _SOFT_UPPER_CONFIG{
	int port_name_index;
	int baudrate_index;

	bool gx;
	bool gy;
	bool gz;
	bool ax;
	bool ay;
	bool az;
	bool roll_sel;
	bool pitch_sel;
	bool yaw_sel;
	bool mx_sel;
	bool my_sel;
	bool mz_sel;
	bool h_sel;
	bool acc_sel;
	bool mag_sel;
	bool est_roll_sel;
	bool est_pitch_sel;
	int8_t unit_rad;
}SOFT_UPPER_CONFIG;

typedef struct _LINE_DATA{
	QwtPlotCurve *Curve = NULL;
	QVector<double> y_data;
}LINE_DATA;

//typedef struct _

typedef struct _CAN_Rx_MSG{
  uint8_t M0_state;
  uint8_t reserve0;
  int16_t data0;
  uint8_t M1_state;
  uint8_t reserve1;
  int16_t data1;
}CAN_Rx_MSG;

class dual_foc : public QMainWindow
{
	Q_OBJECT

public:
	explicit dual_foc(QWidget *parent = 0);
	~dual_foc();
	void System_Init(void);

private slots:
	void on_uart_open_button_clicked();
	void Update_timer_handler(void);

	void on_Port_name_currentIndexChanged(int index);

	void on_roll_select_stateChanged(int arg1);

	void on_pitch_selsect_stateChanged(int arg1);

	void on_yaw_selsect_stateChanged(int arg1);

	void on_save_data_button_clicked();

	void on_mz_select_stateChanged(int arg1);

	void on_mx_select_stateChanged(int arg1);

	void on_my_select_stateChanged(int arg1);

	void on_baudrate_currentIndexChanged(int index);

	void on_set_button_clicked();

	void on_h_select_stateChanged(int arg1);

    void on_start_mag_cal_button_clicked();

    void on_write_mag_cal_button_clicked();

	void on_rad_sw_toggled(bool checked);

	void on_gz_select_stateChanged(int arg1);

	void on_gx_select_stateChanged(int arg1);

	void on_gy_select_stateChanged(int arg1);

	void on_ax_select_stateChanged(int arg1);

	void on_ay_select_stateChanged(int arg1);

	void on_az_select_stateChanged(int arg1);

	void on_gyro_offset_button_clicked();

	void on_acc_offset_botton_clicked();

	void on_acc_select_stateChanged(int arg1);

	void on_mag_select_stateChanged(int arg1);

	void on_est_roll_select_stateChanged(int arg1);

	void on_est_pitch_select_stateChanged(int arg1);

private:
	Ui::dual_foc *ui;
	uint8_t port_flag;
    bool mag_cal_flag;
    float mag_scale[3][2];
    uint16_t mag_cnt;
    QwtPlotCurve* mag_curve[3];
    QwtPlotCurve* StdCir_curve[3];
	uint8_t motor_state;
	CAN_Rx_MSG can_command;
	MOTOR_COMMOND uart_command;
	bool send_command = true;

	SOFT_UPPER_CONFIG upper_config;
	FILE *config_file;

	QTimer *update_timer;

	QButtonGroup *Rad_Degree_sw;

	QVector<double> x_data;

    QVector<double> mag_data[3];
    float mag_offset[3];
	LINE_DATA Line[DIS_LINE_NUM];
	uint8_t line_flag[DIS_LINE_NUM];

	void Zeng_Curve_init(LINE_DATA *Curve, uint16_t thumb);
	void Zeng_Curve_clean(LINE_DATA *Curve);
	void Zeng_Curve_display(void);
	void Zeng_curve_update(LINE_DATA *curve, double data);
	void FOC_display_message(QString message);
	void Zeng_SW_button_state(bool state);
	void Zeng_uart_command_header_init(uint8_t type);

	void config_file_init(void);
	void config_file_save(void);
};

#endif // DUAL_FOC_H
