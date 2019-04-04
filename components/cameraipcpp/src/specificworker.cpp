/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	MemoryStruct chunk{(char *)malloc(1), 0, 0, 0};
	curl_global_init(CURL_GLOBAL_ALL);
	curl_handle = curl_easy_init();
	curl_easy_setopt(curl_handle, CURLOPT_URL, URL);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &chunk);
	curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");

	//this->Period = period;
	timer.setSingleShot(true);
	timer.start(Period);
}

void SpecificWorker::compute()
{
	//computeCODE

	res = curl_easy_perform(curl_handle);
    
    if (res != CURLE_OK)
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    else
        printf("%lu bytes retrieved\n", (unsigned long)chunk.size);

    curl_easy_cleanup(curl_handle);
    free(chunk.memory);
    curl_global_cleanup();
}

static size_t
WriteMemoryCallback(void *contents, size_t size, size_t nmemb, MemoryStruct *mem)
{
    size_t realsize = size * nmemb;
    size_t current = mem->size;
    mem->memory  = (char *)realloc(mem->memory, mem->size + realsize + 1);
    mem->size += realsize;
    
    if (mem->memory == nullptr)
    {
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }

    memcpy(mem->memory + current, contents, realsize);
    //mem->memory[mem->size] = 0;

    if (mem->size < 1024)
        return realsize;

    if (mem->begin == 0)
    {
        for (size_t i = 0; i < mem->size - 1; ++i)
            if ((unsigned char)mem->memory[i] == 0xFF && (unsigned char)mem->memory[i + 1] == 0xD8)
                mem->begin = i;
    }
    else  // end of frame
        for(size_t j = 0; j < realsize; ++j)
            if (*((unsigned char *)contents + j) == 0xff && *((unsigned char *)contents + j + 1) == 0xD9)
                mem->end = current + j + 2;

    if (mem->end != 0)
    {
        std::vector<uchar> buf;
        buf.assign(mem->memory + mem->begin, mem->memory + mem->end);
        cv::Mat img = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
        cv::imshow("Camara sala de reuniones", img);
        cvWaitKey(1);

        size_t nbytes = mem->size - mem->end;
        char *tmp = (char *)malloc(nbytes);
        memcpy(tmp, mem->memory + mem->end, nbytes);
        free(mem->memory);
        mem->size = 0;
        mem->begin = mem->end = 0;
        mem->memory = tmp;
    }
    return realsize;
}

/////////////////////////////////////////////////////////////////////77
////  SERVANT
///////////////////////////////////////////////////////////////////////

void SpecificWorker::CameraSimple_getImage(TImage &im)
{
  //implementCODE

}


