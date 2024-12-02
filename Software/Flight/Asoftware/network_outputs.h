// Network Outputs
const char* NWO_HEADER = (
	"time [s],"
	"state [-],"
	"LOAD-1 [lbf],"
	"LOAD-2 [lbf],"
	"LOAD-3 [lbf],"
	"LOAD-4 [lbf],"
	"LOAD-5 [lbf],"
	"LOAD-6 [lbf],"
	"MOTOR-1 [-],"
	"MOTOR-2 [-]"
);
float NWO_time__s = -404.0;
float NWO_state__ = -404.0;
float NWO_LOAD_1__lbf = -404.0;
float NWO_LOAD_2__lbf = -404.0;
float NWO_LOAD_3__lbf = -404.0;
float NWO_LOAD_4__lbf = -404.0;
float NWO_LOAD_5__lbf = -404.0;
float NWO_LOAD_6__lbf = -404.0;
float NWO_MOTOR_1__ = -404.0;
float NWO_MOTOR_2__ = -404.0;


// Network Inputs
enum NWI{
	NWI_ERROR,
	NWI_START,
	NWI_STOP,
	NWI_ABORT,
	NWI_TERMINATE,
	NWI_UNKNOWN
};


