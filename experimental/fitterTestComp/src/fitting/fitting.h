#ifndef OROL_FITTING
#define OROL_FITTING

#include <boost/signals2.hpp>

/** \brief Generic clas for fitting algorithms
  * \author Marco A. Gutierrez <marcog@unex.es>
  * \ingroup fitting
  */

class fitting
{
  
public:
  /** \brief Constructor. */
  fitting () : running(false) {};
      
  inline bool isRunning () { return running; }
  
  inline void start () { running=true; }
  inline void stop () { running=false; }
  
  template<typename T> boost::signals2::connection registerCallback (const boost::function<T> & callback);
  
protected:
  
  std::map<std::string, boost::signals2::signal_base*> signals_;
  std::map<std::string, std::vector<boost::signals2::connection> > connections;
  std::map<std::string, std::vector<boost::signals2::shared_connection_block> > shared_connections;
  bool running;
  
  template<typename T> boost::signals2::signal<T>* createSignal ();
  
  template<typename T> int num_slots () const;
  
};


template<typename T> int
fitting::num_slots () const
{
  typedef boost::signals2::signal<T> Signal;

  // see if we have a signal for this type
  std::map<std::string, boost::signals2::signal_base*>::const_iterator signal_it = signals_.find (typeid (T).name ());
  if (signal_it != signals_.end ())
  {
    Signal* signal = dynamic_cast<Signal*> (signal_it->second);
    return (static_cast<int> (signal->num_slots ()));
  }
  return (0);
}


template<typename T> boost::signals2::signal<T>*
fitting::createSignal ()
{
  typedef boost::signals2::signal<T> Signal;

  if (signals_.find (typeid (T).name ()) == signals_.end ())
  {
    Signal* signal = new Signal ();
    signals_[typeid (T).name ()] = signal;
    return (signal);
  }
  return (0);

}

template<typename T> boost::signals2::connection
fitting::registerCallback (const boost::function<T> & callback)
{
  typedef boost::signals2::signal<T> Signal;
  if (signals_.find (typeid (T).name ()) == signals_.end ())
  {

    std::cout << "no callback for type:" << typeid (T).name ();
  }
  Signal* signal = dynamic_cast<Signal*> (signals_[typeid (T).name ()]);
  boost::signals2::connection ret = signal->connect (callback);

  connections[typeid (T).name ()].push_back (ret);
  shared_connections[typeid (T).name ()].push_back (boost::signals2::shared_connection_block (connections[typeid (T).name ()].back (), false));
 //signalsChanged ();
  return (ret);
}

#endif