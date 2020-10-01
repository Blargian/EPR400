#include "temperatureSensor.h"
#include "math.h"

#define TABLE_LEN 76

//Place NTC 100k thermistor lookup table generated in Excel here.

thermistor_lookUp NTC100K[TABLE_LEN] = {{.x=3871.804,.y=0},
		{.x=3821.623,.y=1},
		{.x=3770.497,.y=2},
		{.x=3718.464,.y=3},
		{.x=3665.568,.y=4},
		{.x=3611.853,.y=5},
		{.x=3557.178,.y=6},
		{.x=3501.764,.y=7},
		{.x=3445.697,.y=8},
		{.x=3389,.y=9},
		{.x=3331.738,.y=10},
		{.x=3272.875,.y=11},
		{.x=3213.541,.y=12},
		{.x=3192.408,.y=13},
		{.x=3093.716,.y=14},
		{.x=3033.352,.y=15},
		{.x=2972.854,.y=16},
		{.x=2912.208,.y=17},
		{.x=2851.479,.y=18},
		{.x=2790.731,.y=19},
		{.x=2730.028,.y=20},
		{.x=2668.605,.y=21},
		{.x=2607.349,.y=22},
		{.x=2546.323,.y=23},
		{.x=2485.586,.y=24},
		{.x=2425.198,.y=25},
		{.x=2367.128,.y=26},
		{.x=2309.498,.y=27},
		{.x=2252.354,.y=28},
		{.x=2195.738,.y=29},
		{.x=2139.694,.y=30},
		{.x=2082.57,.y=31},
		{.x=2026.124,.y=32},
		{.x=1970.391,.y=33},
		{.x=1915.408,.y=34},
		{.x=1861.206,.y=35},
		{.x=1807.488,.y=36},
		{.x=1754.613,.y=37},
		{.x=1702.607,.y=38},
		{.x=1651.49,.y=39},
		{.x=1601.284,.y=40},
		{.x=1551.437,.y=41},
		{.x=1502.548,.y=42},
		{.x=1454.626,.y=43},
		{.x=1407.681,.y=44},
		{.x=1361.717,.y=45},
		{.x=1315.789,.y=46},
		{.x=1270.892,.y=47},
		{.x=1227.018,.y=48},
		{.x=1184.171,.y=49},
		{.x=1142.345,.y=50},
		{.x=1104.17,.y=51},
		{.x=1066.892,.y=52},
		{.x=1030.506,.y=53},
		{.x=995,.y=54},
		{.x=960.367,.y=55},
		{.x=925.155,.y=56},
		{.x=890.856,.y=57},
		{.x=857.458,.y=58},
		{.x=824.95,.y=59},
		{.x=793.317,.y=60},
		{.x=762.107,.y=61},
		{.x=731.761,.y=62},
		{.x=702.262,.y=63},
		{.x=673.594,.y=64},
		{.x=645.741,.y=65},
		{.x=618.564,.y=66},
		{.x=592.169,.y=67},
		{.x=566.544,.y=68},
		{.x=541.665,.y=69},
		{.x=517.519,.y=70},
		{.x=493.615,.y=71},
		{.x=470.431,.y=72},
		{.x=447.948,.y=73},
		{.x=426.148,.y=74},
		{.x=405.017,.y=75}
		};

float interpolate_table (const thermistor_lookUp * table, float adc_read, int size){
	int i;
	float m;
	i=0;

	// Find two points to work with.
	while((i<size) && (adc_read < table[i].x)){
		i++;
	}

	//If the position is 0, ensure that the first table value is returned.

	if(i ==0){
		return table[i].y;
	}

	//If the position is the very last in the table, ensure that it is taken.
	if(i == size){
		return table[i-1].y;
	}

	//For all other cases, do the interpolation m = (y2-y2)/(x2-x1)

	m = (table[i].y-table[i-1].y)/(table[i].x-table[i-1].x);

	//Return the solution y=m(x-x1)+y1 where x = adc_read

	return m*(adc_read-table[i].x)+table[i].y;

}

uint16_t read_temp(uint16_t sample){

     float temp;
     temp = interpolate_table(NTC100K, (float)sample, TABLE_LEN);
     return (uint16_t) temp;
}

float getTemperature(uint16_t adcValue){
	float y = -0.000000001540*pow(adcValue,3) + 0.000012241208*pow(adcValue,2) - 0.048113478412*adcValue + 91.639615492516; //Third order polynomial fit obtained in Excel
	return y;
}
