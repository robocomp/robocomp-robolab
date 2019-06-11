#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

//g++ -O3 -mavx2 getinmemory.c -o getinmemory -lopencv_highgui -lopencv_imgcodecs  -lopencv_core -lcurl 

//#define URL "http://192.168.0.100:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00"
#define URL "http://10.253.247.24:88/cgi-bin/CGIStream.cgi?cmd=GetMJStream&usr=guest&pwd=smpt00"

struct MemoryStruct
{
    char *memory;
    size_t size;
    size_t begin, end;
};

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

int main(void)
{
    CURL *curl_handle;
    CURLcode res;
    MemoryStruct chunk{(char *)malloc(1), 0, 0, 0};

    curl_global_init(CURL_GLOBAL_ALL);
    curl_handle = curl_easy_init();
    curl_easy_setopt(curl_handle, CURLOPT_URL, URL);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &chunk);
    curl_easy_setopt(curl_handle, CURLOPT_USERAGENT, "libcurl-agent/1.0");
    res = curl_easy_perform(curl_handle);
    
    if (res != CURLE_OK)
        fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
    else
        printf("%lu bytes retrieved\n", (unsigned long)chunk.size);

    curl_easy_cleanup(curl_handle);
    free(chunk.memory);
    curl_global_cleanup();

    return 0;
}
