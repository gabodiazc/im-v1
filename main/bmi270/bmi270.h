#ifndef BMI270
#define BMI270

void readout_data_bmi270(Measure *measure, bool loop, bool act_acc, bool act_gyr);
void bmi270_init(int acc_odr, int acc_avg, int acc_range, int gyr_odr, int gyr_range);

#endif