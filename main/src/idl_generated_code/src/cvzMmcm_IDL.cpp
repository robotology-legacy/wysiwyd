// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <cvzMmcm_IDL.h>
#include <yarp/os/idl/WireTypes.h>



class cvzMmcm_IDL_start : public yarp::os::Portable {
public:
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_pause : public yarp::os::Portable {
public:
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_setLearningRate : public yarp::os::Portable {
public:
  double l;
  void init(const double l);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_getLearningRate : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_setSigma : public yarp::os::Portable {
public:
  double s;
  void init(const double s);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_getSigma : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_getActivity : public yarp::os::Portable {
public:
  int32_t x;
  int32_t y;
  int32_t z;
  double _return;
  void init(const int32_t x, const int32_t y, const int32_t z);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_saveWeightsToFile : public yarp::os::Portable {
public:
  std::string path;
  bool _return;
  void init(const std::string& path);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_loadWeightsFromFile : public yarp::os::Portable {
public:
  std::string path;
  bool _return;
  void init(const std::string& path);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class cvzMmcm_IDL_saveRF : public yarp::os::Portable {
public:
  std::string path;
  bool _return;
  void init(const std::string& path);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool cvzMmcm_IDL_start::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("start",1,1)) return false;
  return true;
}

bool cvzMmcm_IDL_start::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

void cvzMmcm_IDL_start::init() {
}

bool cvzMmcm_IDL_pause::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("pause",1,1)) return false;
  return true;
}

bool cvzMmcm_IDL_pause::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

void cvzMmcm_IDL_pause::init() {
}

bool cvzMmcm_IDL_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool cvzMmcm_IDL_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_quit::init() {
  _return = false;
}

bool cvzMmcm_IDL_setLearningRate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setLearningRate",1,1)) return false;
  if (!writer.writeDouble(l)) return false;
  return true;
}

bool cvzMmcm_IDL_setLearningRate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

void cvzMmcm_IDL_setLearningRate::init(const double l) {
  this->l = l;
}

bool cvzMmcm_IDL_getLearningRate::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getLearningRate",1,1)) return false;
  return true;
}

bool cvzMmcm_IDL_getLearningRate::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_getLearningRate::init() {
  _return = (double)0;
}

bool cvzMmcm_IDL_setSigma::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setSigma",1,1)) return false;
  if (!writer.writeDouble(s)) return false;
  return true;
}

bool cvzMmcm_IDL_setSigma::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  return true;
}

void cvzMmcm_IDL_setSigma::init(const double s) {
  this->s = s;
}

bool cvzMmcm_IDL_getSigma::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getSigma",1,1)) return false;
  return true;
}

bool cvzMmcm_IDL_getSigma::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_getSigma::init() {
  _return = (double)0;
}

bool cvzMmcm_IDL_getActivity::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("getActivity",1,1)) return false;
  if (!writer.writeI32(x)) return false;
  if (!writer.writeI32(y)) return false;
  if (!writer.writeI32(z)) return false;
  return true;
}

bool cvzMmcm_IDL_getActivity::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_getActivity::init(const int32_t x, const int32_t y, const int32_t z) {
  _return = (double)0;
  this->x = x;
  this->y = y;
  this->z = z;
}

bool cvzMmcm_IDL_saveWeightsToFile::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("saveWeightsToFile",1,1)) return false;
  if (!writer.writeString(path)) return false;
  return true;
}

bool cvzMmcm_IDL_saveWeightsToFile::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_saveWeightsToFile::init(const std::string& path) {
  _return = false;
  this->path = path;
}

bool cvzMmcm_IDL_loadWeightsFromFile::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("loadWeightsFromFile",1,1)) return false;
  if (!writer.writeString(path)) return false;
  return true;
}

bool cvzMmcm_IDL_loadWeightsFromFile::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_loadWeightsFromFile::init(const std::string& path) {
  _return = false;
  this->path = path;
}

bool cvzMmcm_IDL_saveRF::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("saveRF",1,1)) return false;
  if (!writer.writeString(path)) return false;
  return true;
}

bool cvzMmcm_IDL_saveRF::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void cvzMmcm_IDL_saveRF::init(const std::string& path) {
  _return = false;
  this->path = path;
}

cvzMmcm_IDL::cvzMmcm_IDL() {
  yarp().setOwner(*this);
}
void cvzMmcm_IDL::start() {
  cvzMmcm_IDL_start helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void cvzMmcm_IDL::start()");
  }
  yarp().write(helper,helper);
}
void cvzMmcm_IDL::pause() {
  cvzMmcm_IDL_pause helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void cvzMmcm_IDL::pause()");
  }
  yarp().write(helper,helper);
}
bool cvzMmcm_IDL::quit() {
  bool _return = false;
  cvzMmcm_IDL_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool cvzMmcm_IDL::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void cvzMmcm_IDL::setLearningRate(const double l) {
  cvzMmcm_IDL_setLearningRate helper;
  helper.init(l);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void cvzMmcm_IDL::setLearningRate(const double l)");
  }
  yarp().write(helper,helper);
}
double cvzMmcm_IDL::getLearningRate() {
  double _return = (double)0;
  cvzMmcm_IDL_getLearningRate helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double cvzMmcm_IDL::getLearningRate()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
void cvzMmcm_IDL::setSigma(const double s) {
  cvzMmcm_IDL_setSigma helper;
  helper.init(s);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","void cvzMmcm_IDL::setSigma(const double s)");
  }
  yarp().write(helper,helper);
}
double cvzMmcm_IDL::getSigma() {
  double _return = (double)0;
  cvzMmcm_IDL_getSigma helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double cvzMmcm_IDL::getSigma()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double cvzMmcm_IDL::getActivity(const int32_t x, const int32_t y, const int32_t z) {
  double _return = (double)0;
  cvzMmcm_IDL_getActivity helper;
  helper.init(x,y,z);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double cvzMmcm_IDL::getActivity(const int32_t x, const int32_t y, const int32_t z)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool cvzMmcm_IDL::saveWeightsToFile(const std::string& path) {
  bool _return = false;
  cvzMmcm_IDL_saveWeightsToFile helper;
  helper.init(path);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool cvzMmcm_IDL::saveWeightsToFile(const std::string& path)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool cvzMmcm_IDL::loadWeightsFromFile(const std::string& path) {
  bool _return = false;
  cvzMmcm_IDL_loadWeightsFromFile helper;
  helper.init(path);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool cvzMmcm_IDL::loadWeightsFromFile(const std::string& path)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool cvzMmcm_IDL::saveRF(const std::string& path) {
  bool _return = false;
  cvzMmcm_IDL_saveRF helper;
  helper.init(path);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool cvzMmcm_IDL::saveRF(const std::string& path)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool cvzMmcm_IDL::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "start") {
      start();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "pause") {
      pause();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setLearningRate") {
      double l;
      if (!reader.readDouble(l)) {
        reader.fail();
        return false;
      }
      setLearningRate(l);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getLearningRate") {
      double _return;
      _return = getLearningRate();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setSigma") {
      double s;
      if (!reader.readDouble(s)) {
        reader.fail();
        return false;
      }
      setSigma(s);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(0)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getSigma") {
      double _return;
      _return = getSigma();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getActivity") {
      int32_t x;
      int32_t y;
      int32_t z;
      if (!reader.readI32(x)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(y)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(z)) {
        reader.fail();
        return false;
      }
      double _return;
      _return = getActivity(x,y,z);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "saveWeightsToFile") {
      std::string path;
      if (!reader.readString(path)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = saveWeightsToFile(path);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "loadWeightsFromFile") {
      std::string path;
      if (!reader.readString(path)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = loadWeightsFromFile(path);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "saveRF") {
      std::string path;
      if (!reader.readString(path)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = saveRF(path);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> cvzMmcm_IDL::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("start");
    helpString.push_back("pause");
    helpString.push_back("quit");
    helpString.push_back("setLearningRate");
    helpString.push_back("getLearningRate");
    helpString.push_back("setSigma");
    helpString.push_back("getSigma");
    helpString.push_back("getActivity");
    helpString.push_back("saveWeightsToFile");
    helpString.push_back("loadWeightsFromFile");
    helpString.push_back("saveRF");
    helpString.push_back("help");
  }
  else {
    if (functionName=="start") {
      helpString.push_back("void start() ");
      helpString.push_back("Start the computation of predictions trying to cope with the period. ");
    }
    if (functionName=="pause") {
      helpString.push_back("void pause() ");
      helpString.push_back("Pause the computation of predictions ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
      helpString.push_back("@return true/false on success/failure. ");
    }
    if (functionName=="setLearningRate") {
      helpString.push_back("void setLearningRate(const double l) ");
      helpString.push_back("Set the learning rate. ");
    }
    if (functionName=="getLearningRate") {
      helpString.push_back("double getLearningRate() ");
      helpString.push_back("Get the current learning rate. ");
    }
    if (functionName=="setSigma") {
      helpString.push_back("void setSigma(const double s) ");
      helpString.push_back("Set the neighborhood radius (sigma). ");
    }
    if (functionName=="getSigma") {
      helpString.push_back("double getSigma() ");
      helpString.push_back("Get the current neighborhood radius (sigma). ");
    }
    if (functionName=="getActivity") {
      helpString.push_back("double getActivity(const int32_t x, const int32_t y, const int32_t z) ");
      helpString.push_back("Get the activity of a neuron. ");
      helpString.push_back("@param x the horizontal coordinate of the neuron. ");
      helpString.push_back("@param y the vertical coordinate of the neuron. ");
      helpString.push_back("@param z the layer to which the neuron belongs. ");
      helpString.push_back("@return Current activity of the neuron. ");
    }
    if (functionName=="saveWeightsToFile") {
      helpString.push_back("bool saveWeightsToFile(const std::string& path) ");
      helpString.push_back("Save the current weights of the map for latter reloading. ");
      helpString.push_back("@param path The path to where you want to store the weights. ");
      helpString.push_back("@return true/false in case of success/failure. ");
    }
    if (functionName=="loadWeightsFromFile") {
      helpString.push_back("bool loadWeightsFromFile(const std::string& path) ");
      helpString.push_back("Load the weights of the map from a file. ");
      helpString.push_back("The file should have been saved from a map using the same config file (modaility names & size, map size, etc.). ");
      helpString.push_back("@param path Path to the file containing the weights. ");
      helpString.push_back("@return true/false in case of success/failure. ");
    }
    if (functionName=="saveRF") {
      helpString.push_back("bool saveRF(const std::string& path) ");
      helpString.push_back("Save the receptive fields of a map. A different file will be produced for each modality. ");
      helpString.push_back("Produces an image ");
      helpString.push_back("@param path Path to the file that will receive the RF without extension (.jpg). ");
      helpString.push_back("@return true/false in case of success/failure. ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


