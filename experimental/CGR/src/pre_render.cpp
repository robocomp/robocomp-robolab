//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    pre_render.cpp
\brief   A utility to pre-render a map
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <vector>
#include <pthread.h>

#include "vector_map.h"
#include "popt_pp.h"
#include "terminal_utils.h"
#include "timer.h"
#include "proghelp.h"

using namespace std;


bool run = true;
int numSteps = 0;
int debugLevel = 0;
float minRange = 0.000001;
float maxRange = 8.0;
double resolution = 0.2;

VectorMap *vectorMap;

typedef struct{
  int taskIndex;
  int taskStart;
  int taskEnd;
} task_t;

vector<vector<int> > preRenderList;
//Synchronization for threads
int numThreads = 1;
pthread_t* threads;
vector<vector<int> >** sceneLines;
vector<vector2i>** sceneLocations;
int* threadProgress;

vector<int> getUniqueValues(vector<int> &v)
{
  vector<int> ret;
  ret.clear();
  for(int i=0; i<(int)v.size(); i++){
    if(v[i]<0)
      continue;
    bool found = false;
    for(int j=0; !found && j<(int)ret.size(); j++){
      found = (v[i]==ret[j]);
    }
    if(!found)
      ret.push_back(v[i]);
  }
  return ret;
}

void* PreRenderThread(void* arg)
{
  static const bool BlankTest = false;
  
  task_t subTask = *((task_t*) arg);
  
  if(debugLevel>2) printf("Thread %d sub-task: %d - %d\n",subTask.taskIndex, subTask.taskStart,subTask.taskEnd);
  
  int w = ceil((vectorMap->maxX-vectorMap->minX)/resolution);
  vector2f loc;
  vector<vector<int> > * mySceneLines = new vector<vector<int> >();
  mySceneLines->clear();
  sceneLines[subTask.taskIndex] = mySceneLines;
  vector<vector2i> * mylocations = new vector<vector2i>();
  mylocations->clear();
  sceneLocations[subTask.taskIndex] = mylocations;
  
  threadProgress[subTask.taskIndex] = subTask.taskStart;
  for(int i=subTask.taskStart; i<=subTask.taskEnd; i++){
    double y = double(i)*resolution + vectorMap->minY;
    for(int j=0; j<w; j++){
      double x = double(j)*resolution + vectorMap->minX;
      loc.set(x,y);
      mylocations->push_back(vector2i(j,i));
      if(BlankTest){
        vector<int> list;
        list.push_back(1);
        list.push_back(0);
        mySceneLines->push_back(list); 
      }else{
        //mySceneLines->push_back(vectorMap->getSceneLines(loc,0.0,RAD(360),RAD(0.5),minRange,maxRange));
        mySceneLines->push_back(vectorMap->getSceneLines(loc,maxRange));
      }
      
      threadProgress[subTask.taskIndex] = i+1;
    }
  }
  return NULL;
}

void PreRenderMultiThread()
{
  threads = (pthread_t*) malloc(numThreads*sizeof(pthread_t));
  
  int w = ceil((vectorMap->maxX-vectorMap->minX)/resolution);
  int h = ceil((vectorMap->maxY-vectorMap->minY)/resolution);
  vector2d loc;
  
  preRenderList.clear();
  
  printf("\nPre-Render Using %d threads.\n",numThreads);
  printf("Size: %d x %d\nResolution:%.3f\n",w, h, resolution);
  task_t subTasks[numThreads];
  int start = 0;
  int taskIncrement = h/numThreads;
  for(int i=0; i<numThreads; i++){
    subTasks[i].taskIndex = i;
    subTasks[i].taskStart = start;
    start += taskIncrement;
    subTasks[i].taskEnd = start-1;
  }
  subTasks[numThreads-1].taskEnd = h-1;
  
  //Allocate
  sceneLines = (vector<vector<int> > **) malloc(numThreads*sizeof(vector<vector<int> > *));
  sceneLocations = (vector<vector2i> **) malloc(numThreads*sizeof(vector<vector2i> *));
  threadProgress = (int*) malloc(numThreads*sizeof(int));
  
  //Fork
  double tStart = GetTimeSec();
  for(int i=0; i<numThreads; i++){
    pthread_create( &threads[i], NULL, PreRenderThread, (void*) &subTasks[i]);
  }
  
  //Monitor Progress
  bool done=false;
  while(!done){
    Sleep(0.1);
    done = true;
    double totalProgress = 0.0;
    printf("\rThread Progress: ");
    for(int i=0;i<numThreads; i++){
      printf("%6.2f%%   ",double(threadProgress[i]-subTasks[i].taskStart)/double(subTasks[i].taskEnd-subTasks[i].taskStart+1)*100.0);
      //printf("%6.2f%%   ",double(threadProgress[i]));
      done = done && (threadProgress[i]>subTasks[i].taskEnd);
      totalProgress += double(threadProgress[i]-subTasks[i].taskStart)/double(subTasks[i].taskEnd-subTasks[i].taskStart+1)*100.0;
    }
    printf("Total: %6.2f%%  ",totalProgress/double(numThreads));
    fflush(stdout);
  }
  
  //Join
  for(int i=0; i<numThreads; i++){
    pthread_join(threads[i],NULL);
  }
  
  double tDuration = GetTimeSec()-tStart;
  printf("\nDone Rendering in %.3fs, Saving to file...\n", tDuration);
  FILE* fidstats = fopen("pre_render_stats.txt","a");
  fprintf(fidstats, "%d, %d, %f\n",h*w,numThreads,tDuration);
  fclose(fidstats);
  
  //Combine all the data
  static const bool debugLines = false;
  char renderFile[4096];
  snprintf(renderFile, 4095,"./maps/%s/%s_render.dat",vectorMap->mapName.c_str(),vectorMap->mapName.c_str());
  FILE* fid = fopen(renderFile,"w+");
  printf("Saving to %s\n",renderFile);

  fwrite(&w,sizeof(int),1,fid);
  fwrite(&h,sizeof(int),1,fid);
  fwrite(&resolution,sizeof(double),1,fid);
  for(int i=0; i<numThreads;i++){
    const vector<vector<int> > &partScene = *sceneLines[i];
    int numLocs = partScene.size();
    for(int j=0; j<numLocs; j++){
      int x = sceneLocations[i]->at(j).x;
      int y = sceneLocations[i]->at(j).y;
      if(debugLines){
        printf("loc:%5d,%5d, lines:",x,y);
        for(unsigned int k=0; k<partScene[j].size(); k++){
          printf(" %3d", partScene[j][k]);
        }
        printf("\n");
      }
      fwrite(&x,sizeof(int),1,fid);
      fwrite(&y,sizeof(int),1,fid);
      int size = partScene[j].size();
      fwrite(&size,sizeof(int),1,fid);
      fwrite(partScene[j].data(),sizeof(int),size,fid);
    }
  }
}

int main(int argc, char** argv)
{  
  //========================== Set up Command line parameters =======================
  char *map_name = (char*) malloc(4096);
  snprintf(map_name, 4095, "GHC7");
  
  static struct poptOption options[] = {
    { "map-name",         'm', POPT_ARG_STRING , &map_name,       0, "Map name",              "STRING"},
    { "debug",            'd', POPT_ARG_INT,     &debugLevel,     0, "Debug Level",           "NUM"},
    { "num-threads",      'n', POPT_ARG_INT,     &numThreads,     0, "Num Threads",           "NUM"},
    { "render-resolution",'r', POPT_ARG_DOUBLE,  &resolution,     0, "Render Resolution",     "NUM"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  //========================= Welcome screen, Load map & log ========================
  ColourTerminal(TerminalUtils::TERMINAL_COL_GREEN,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nVector Localization Pre-Render Optimization\n\n");
  ResetTerminal();
  
  printf("Map: %s\n",map_name);
  printf("Num Threads: %d\n",numThreads);
  //Read Map
  vectorMap = new VectorMap(map_name,"./maps",false);
  
  InitHandleStop(&run);
  PreRenderMultiThread();
  
  printf("closing.\n");
  return 0;
}
