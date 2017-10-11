#ifndef __UTILS_H_INCLUDED__
#define __UTILS_H_INCLUDED__

#include <chrono>
using namespace std::chrono;

static long double ant  = 0.0;
static long double now  = 0.0;
static long double cont = 0.0;
static long double sum  = 0.0;
static long double mean = 0.0;
static long double max = 0.0;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window limit";

typedef struct Figure{
    double width;
    double height;
    double r;
}figure;

/*Calc T in nanosecond ^-9*/
void inline calc_T(){
	now = std::chrono::duration_cast<std::chrono::nanoseconds>
					(std::chrono::system_clock::now().time_since_epoch()).count();
	
	if(ant){
		cont++;
		sum += (now - ant);
		max = ((now-ant) > max) ? now-ant : max;
		mean = sum/cont;
		std::cout	<< "value: " << (now - ant) << std::endl
					<< "mean:  " << mean << std::endl
					<< "max:   " << max << std::endl << std::endl;
	}
	ant = now;	
}

#endif

