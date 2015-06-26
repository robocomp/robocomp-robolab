#include <sys/types.h>
#include <sys/stat.h>

#include <math.h>
#include <stdio.h>

extern "C" {
#include <lua5.1/lua.h>
#include <lua5.1/lualib.h>
#include <lua5.1/lauxlib.h>
};

#include "ansicolor.h"
#include "util.h"

#include "configreader.h"

static const bool Debug = false;

/*
  [ References ]

  Lua for configuration files:
    Programming in  Lua
    Chapter 25. Extending your Application
    http://www.lua.org/pil/25.html
  Lua C API function documentation:
    Lua 5.0 Reference Manual
    Chapter 3 - The Application Program Interface
    http://www.lua.org/manual/5.0/manual.html#3
*/

//====================================================================//

bool FileExists(const char *filename)
{
  struct stat st;
  return(stat(filename,&st) == 0);
}

//====================================================================//

ConfigReader::SubTree::SubTree(ConfigReader &c,const char *base_exp)
{
  config = &c;
  base = base_exp;
  errors_at_init = config->errors;
}

ConfigReader::SubTree::SubTree(SubTree &s,const char *base_exp)
{
  config = s.config;
  const char *sep = (base_exp[0]=='[')? "" : ".";
  base.printf("%s%s%s",s.base(),sep,base_exp);
  errors_at_init = config->errors;
}

const char *ConfigReader::SubTree::getFullExp(const char *exp)
{
  const char *sep = (exp[0]=='[')? "" : ".";
  full.printf("%s%s%s",base(),sep,exp);
  return(full());
}

const char *ConfigReader::SubTree::getStr(const char *exp,
                                          const char *default_val)
{
  return(config->getStr(getFullExp(exp),default_val));
}

bool ConfigReader::SubTree::getBool(const char *exp,bool &val)
{
  return(config->getBool(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getInt(const char *exp,int &val)
{
  return(config->getInt(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getUInt(const char* exp, unsigned int& val)
{
  return(config->getUInt(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getReal(const char *exp,float  &val)
{
  return(config->getReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getReal(const char *exp,double &val)
{
  return(config->getReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getPosReal(const char *exp,float &val)
{
  return(config->getPosReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getPosReal(const char *exp,double &val)
{
  return(config->getPosReal(getFullExp(exp),val));
}

bool ConfigReader::SubTree::getInt(const char *exp,int &val,
                                   int _min,int _max)
{
  return(config->getInt(getFullExp(exp),val,_min,_max));
}

bool ConfigReader::SubTree::getReal(const char *exp,float &val,
                                    float _min,float _max)
{
  return(config->getReal(getFullExp(exp),val,_min,_max));
}

bool ConfigReader::SubTree::getReal(const char *exp,double &val,
                                    double _min,double _max)
{
  return(config->getReal(getFullExp(exp),val,_min,_max));
}

ConfigReader::FileHeader::FileHeader(const FileHeader &fh)
{
  filename = fh.filename;
  flags = fh.flags;
  watch = fh.watch;
}

//====================================================================//

ConfigReader::ConfigReader()
{
  path = NULL;
  L = NULL;
  errors = 0;
  num_readfiles_calls = 0;
  watch_files = NULL;
  modified = false;
}

ConfigReader::ConfigReader(const char* _path)
{
  path = (char*) malloc(strlen(_path)+1);
  strcpy(path,_path);
  L = NULL;
  errors = 0;
  num_readfiles_calls = 0;
  watch_files = NULL;
  modified = false;
}

bool ConfigReader::initLua()
{
  if(L) closeLua();
  L = lua_open();
  if(!L) return(false);

  // load libraries
  luaL_openlibs(L);

  // discard any results from initialization
  lua_settop(L, 0);

  return(true);
}

void ConfigReader::closeLua()
{
  if(L) lua_close(L);
  L = NULL;
}

void ConfigReader::clearWatches()
{
  for(unsigned i=0; i<files.size(); i++){
    files[i].watch.remove();
  }
}

void ConfigReader::reset()
{
  closeLua();
  clearWatches();
  files.clear();
  num_readfiles_calls = 0;
  watch_files = NULL;
}

void ConfigReader::addFile(const char *filename,unsigned flags)
{
  FileHeader fh;
  if(path!=NULL){
    fh.filename = path;
    fh.filename.add(filename);
  }else{
    fh.filename = filename;
  }
  fh.flags = flags;
  fh.watch.watch(watch_files,filename);
  files.push_back(fh);

  modified = true;
}

void ConfigReader::showError(int err_code,const char *filename)
{
  if(err_code == 0) return;

  AnsiColor::SetFgColor(stderr,AnsiColor::Yellow);
  AnsiColor::Bold(stderr);

  const char *err_str;
  switch(err_code){
    case LUA_ERRFILE:   err_str="File not found"; break;
    case LUA_ERRSYNTAX: err_str="Syntax error"; break;
    case LUA_ERRMEM:    err_str="Memory allocation error"; break;
    case LUA_ERRRUN:    err_str="Runtime error"; break;
    case LUA_ERRERR:    err_str="Error running error handler"; break;
    default: err_str="Uknown error";
  }

  int t = lua_gettop(L);
  if(lua_isstring(L,t)){
    const char *str = lua_tolstring(L,t,NULL);
    fprintf(stderr,"ConfigReader: %s\n",str);
  }

  fprintf(stderr,"ConfigReader: %s: %s\n",filename,err_str);

  AnsiColor::Reset(stderr);
}

bool ConfigReader::readFile(const char *filename,unsigned flags)
{
  if(Debug){
    printf("ConfigReader: reading \"%s\"\n",filename);
  }

  // it's ok if optional files don't exist
  if((flags & Optional)!=0 && !FileExists(filename)) return(true);

  // try to load the file
  int ret = luaL_loadfile(L, filename);
  if(ret){
    showError(ret,filename);
  }else{
    // try to execute the file
    ret = lua_pcall(L, 0, LUA_MULTRET, 0);
    if(ret) showError(ret,filename);
  }

  lua_settop(L, 0); // discard any results

  return(ret == 0);
}

bool ConfigReader::readFiles()
{
  if(!L && !initLua()) return(false);
  errors = 0;
  modified = false;

  bool ok = true;

  for(unsigned i=0; i<files.size(); i++){
    FileHeader &fh = files[i];
    if(fh.watch.isFileModified()){
      fh.watch.rewatch(fh.filename());
    }
    if(!readFile(fh.filename(),fh.flags)){
      ok = false;
    }
  }

  if(ok) num_readfiles_calls++;

  return(ok);
}

bool ConfigReader::isFileModified()
{
  unsigned i=0;
  while(!modified && i<files.size()){
    modified = files[i].watch.isFileModified();
    i++;
  }

  return(modified);
}

bool ConfigReader::needUpdate(int &client_generation) const
{
  if(client_generation != getGeneration()){
    client_generation = getGeneration();
    return(true);
  }else{
    return(false);
  }
}

void ConfigReader::eval(const char *exp)
{
  CharString fexp;
  fexp.printf("_ans=(%s);",exp);
  luaL_dostring(L,fexp());
  lua_getglobal(L,"_ans");
}

void ConfigReader::eval(const char *exp0,const char *exp1)
{
  CharString fexp;
  fexp.printf("_ans=(%s%s);",exp0,exp1);
  luaL_dostring(L,fexp());
  lua_getglobal(L,"_ans");
}

const char *ConfigReader::getStr(const char *exp,const char *default_val)
{
  eval(exp);

  const char *ret = NULL;
  if(lua_isstring(L,-1)){
    ret = lua_tostring(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a string.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ret? ret : default_val);
}

bool ConfigReader::getBool(const char *exp,bool &val)
{
  eval(exp);

  bool ok = lua_isboolean(L,-1);
  if(ok){
    val = (bool)lua_toboolean(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a boolean.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getInt(const char *exp,int &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (int)rint(lua_tonumber(L,-1));
  }else{
    printf("ConfigReader: \"%s\" is not an integer.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getUInt(const char *exp,unsigned int &val)
{
  eval(exp);
  
  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (int)rint(lua_tonumber(L,-1));
    if(val<0){
      printf("ConfigReader: \"%s\" is not an unsigned integer.\n",exp);
      errors++;
      ok = false;
    }
  }else{
    printf("ConfigReader: \"%s\" is not an integer.\n",exp);
    errors++;
  }
  
  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getReal(const char *exp,float &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (float)lua_tonumber(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a real number.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getReal(const char *exp,double &val)
{
  eval(exp);

  bool ok = lua_isnumber(L,-1);
  if(ok){
    val = (float)lua_tonumber(L,-1);
  }else{
    printf("ConfigReader: \"%s\" is not a real number.\n",exp);
    errors++;
  }

  lua_pop(L,-1);
  return(ok);
}

bool ConfigReader::getInt(const char *exp,int &val,int _min,int _max)
{
  if(!getInt(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%d is out of range [%d,%d].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getPosReal(const char *exp,float &val)
{
  if(!getReal(exp,val)) return(false);
  if(val <= 0.0){
    printf("ConfigReader: %s=%f is non-positive\n",exp,val);
    val = 1E-6;
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getPosReal(const char *exp,double &val)
{
  
  if(!getReal(exp,val)){ 
    printf("failure at %s\n",exp);
    return(false);
  }
  if(val <= 0.0){
    printf("ConfigReader: %s=%f is non-positive\n",exp,val);
    val = 1E-6;
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getReal(const char *exp,float &val,float _min,float _max)
{
  if(!getReal(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%f is out of range [%f,%f].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::getReal(const char *exp,double &val,double _min,double _max)
{
  if(!getReal(exp,val)) return(false);
  if(val<_min || val>_max){
    printf("ConfigReader: %s=%f is out of range [%f,%f].\n",
           exp,val,_min,_max);
    val = bound(val,_min,_max);
    errors++;
    return(false);
  }
  return(true);
}

bool ConfigReader::set(const char *name,int val)
{
  CharString fexp;
  fexp.printf("%s=%d;",name,val);
  luaL_dostring(L,fexp());

  int v;
  return(getInt(name,v) && v==val);
}

bool ConfigReader::set(const char *name,double val)
{
  CharString fexp;
  fexp.printf("%s=%f;",name,val);
  luaL_dostring(L,fexp());

  double v;
  return(getReal(name,v) && fabs(val-v)<1E-9);
}

void ConfigReader::addStandard()
{
  // standard includes for a project go here
  addFile("../etc/common.cfg");
}
