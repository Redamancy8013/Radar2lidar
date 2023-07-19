#include <receive/receive_module.h>

using namespace cngicuvc;
int main() 
{ 
  	ReceiveModule module;     
	module.start(1);
	
	while(1)
	{
		dnet_msleepx(100);			
	}	
	module.stop();
	return 0;
}


 
