/*
Source code for the KUKA robot controller communication client.
Author: Eirik B. Njaastad.
NTNU 2015

Communicates with the KUKAVARPROXY server made by
Massimiliano Fago - massimiliano.fago@gmail.com

Modifications:
2016-2017 NTNU - Ivar Eriksen
- Fixed compile error on Linux
- Made all variables class memebers
- Added support function
- Added documentation stubs
*/

#ifndef BOOSTCLIENTCROSS
#define BOOSTCLIENTCROSS

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <string>
#include <vector>

typedef unsigned char BYTE;

/**
 * @brief Class for communication with KUKAVARPROXY running on a KUKA KRC controller
 */
class BoostClientCross
{
private:
  boost::asio::io_service iosClientCross;
  boost::system::error_code socketError;
  boost::asio::ip::tcp::socket* socketClientCross;

public:
  BoostClientCross()
  {
    socketClientCross = new boost::asio::ip::tcp::socket(iosClientCross);
  }

  /**
   * @brief  Function for opening a socket connection and initiate the server connection:
   *
   * @param ipAddress IP address to KRC controller
   * @param portNumber TCP port number, usualy 7000
   */
  void connectSocket(std::string ipAddress, std::string portNumber)
  {
    socketClientCross->connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(ipAddress),
                                                              boost::lexical_cast<unsigned>(portNumber)));
  }

  /**
   * @brief Check that we are connected to robot
   *
   * @return true is socket is open
   */
  bool checkSocket()
  {
    return socketClientCross->is_open();
  }

  /**
   * @brief For writing a variable to the robot controller, the message to send must contain a variable name (varName)
   * and a value to write (varValue).
   *
   * @param varName KRL variable name
   * @param varValue KRL variable value
   * @param messageId KUKAVARPROXY message ID, defaults to 05
   *
   * @return Vector with text formatted to send to KRC
   */
  std::vector<unsigned char> formatWriteMsg(std::vector<unsigned char> varName, std::vector<unsigned char> varValue,
                                            int messageId = 05)
  {
    std::vector<unsigned char> header, block;
    int varNameLength, varValueLength, blockSize;
    //		int messageId;
    BYTE hbyte, lbyte, hbytemsg, lbytemsg;

    varNameLength = varName.size();
    varValueLength = varValue.size();
    //		messageId = 05;

    hbyte = (BYTE)((varNameLength >> 8) & 0xff00);
    lbyte = (BYTE)(varNameLength & 0x00ff);

    block.push_back((unsigned char)1);
    block.push_back((unsigned char)hbyte);
    block.push_back((unsigned char)lbyte);

    for (int i = 0; i != varNameLength; ++i)
    {
      block.push_back(varName[i]);
    }

    hbyte = (BYTE)((varValueLength >> 8) & 0xff00);
    lbyte = (BYTE)(varValueLength & 0x00ff);

    block.push_back((unsigned char)hbyte);
    block.push_back((unsigned char)lbyte);

    for (int i = 0; i != varValueLength; ++i)
    {
      block.push_back(varValue[i]);
    }

    blockSize = block.size();
    hbyte = (BYTE)((blockSize >> 8) & 0xff00);
    lbyte = (BYTE)(blockSize & 0x00ff);

    hbytemsg = (BYTE)((messageId >> 8) & 0xff00);
    lbytemsg = (BYTE)(messageId & 0x00ff);

    header.push_back((unsigned char)hbytemsg);
    header.push_back((unsigned char)lbytemsg);
    header.push_back((unsigned char)hbyte);
    header.push_back((unsigned char)lbyte);

    block.insert(block.begin(), header.begin(), header.end());
    return block;
  }

  /**
   * @brief For reading a variable from the robot controller, the message to send must contain the desired variable name
   * (varName).
   *
   * @param varName KRL Variable to read from
   * @param messageId KUKAVARPROXY message ID
   *
   * @return Vector formated to send to KRC
   */
  std::vector<unsigned char> formatReadMsg(std::vector<unsigned char> varName, int messageId = 05)
  {
    std::vector<unsigned char> header, block;
    int varNameLength, blockSize;
    //		int messageId;
    BYTE hbyte, lbyte, hbytemsg, lbytemsg;

    varNameLength = varName.size();
    //		messageId = 05;

    hbyte = (BYTE)((varNameLength >> 8) & 0xff00);
    lbyte = (BYTE)(varNameLength & 0x00ff);

    block.push_back((unsigned char)0);
    block.push_back((unsigned char)hbyte);
    block.push_back((unsigned char)lbyte);

    for (int i = 0; i != varNameLength; ++i)
    {
      block.push_back(varName[i]);
    }

    blockSize = block.size();

    hbyte = (BYTE)((blockSize >> 8) & 0xff00);
    lbyte = (BYTE)(blockSize & 0x00ff);

    hbytemsg = (BYTE)((messageId >> 8) & 0xff00);
    lbytemsg = (BYTE)(messageId & 0x00ff);

    header.push_back((unsigned char)hbytemsg);
    header.push_back((unsigned char)lbytemsg);
    header.push_back((unsigned char)hbyte);
    header.push_back((unsigned char)lbyte);

    block.insert(block.begin(), header.begin(), header.end());
    return block;
  }

  /**
   * @brief Send the formatted message and recieve server response:
   *
   * @param message Formated message to send. See formatReadMsg and formatWriteMsg
   *
   * @return Response from KRC
   */
  std::vector<unsigned char> sendMsg(std::vector<unsigned char> message)
  {
    // Send message:
    const size_t bytes = boost::asio::write(*socketClientCross, boost::asio::buffer(message));

    // Read answer:
    boost::array<unsigned char, 7> recheader;
    size_t sendLen = socketClientCross->read_some(boost::asio::buffer(recheader), socketError);  // Header
    int messageLength = recheader[3];
    std::vector<unsigned char> recblock(messageLength);
    size_t recLen = socketClientCross->read_some(boost::asio::buffer(recblock), socketError);  // Message

    // Error handling:
    if (socketError == boost::asio::error::eof)
      std::cout << "Connection closed cleanly by peer" << std::endl;
    else if (socketError)
      throw boost::system::system_error(socketError);  // Some other error.

    recblock.erase(recblock.end() - 6, recblock.end());
    return recblock;
  }

  /**
   * @brief Function for terminating the socket and thus disconnect from server
   */
  void disconnectSocket()
  {
    socketClientCross->shutdown(boost::asio::ip::tcp::socket::shutdown_both, socketError);
    socketClientCross->close();

    // Error handling:
    if (socketError)
      throw boost::system::system_error(socketError);
  }
  
  /**
   * @brief Write E6AXIS to KRC.
   *
   * @param write_to KRL E6AXIS variable name
   * @param joint_command Joint values, in order from A1 - A6, E1 - E6
   * @param joints Number of joint values to send
   * @param out Formatted string sent to KRC is stored to this variable
   *
   * @return Currently always returns true. Inteded to return true\false depning on successful write.
   */
  bool writeE6AXIS(const std::string* write_to, const double* joint_command, const std::size_t joints, std::string* out)
  {
    *out = "{E6AXIS: ";

    for (int i = 0; (i < joints) && (i < 6); ++i)
    {
      *out += " A" + std::to_string(i + 1) + ' ' + std::to_string(joint_command[i]) + ',';
    }

    for (int i = 6; (i < joints) && (i < 12); ++i)
    {
      *out += " E" + std::to_string(i - 5) + ' ' + std::to_string(joint_command[i]) + ',';
    }

    out->back() = '}';

    std::vector<unsigned char> out_vector(out->begin(), out->end());
    std::vector<unsigned char> var(write_to->begin(), write_to->end());
    std::vector<unsigned char> formated_out = this->formatWriteMsg(var, out_vector);
    std::vector<unsigned char> reply = this->sendMsg(formated_out);
    // TODO: Check reply from server and return true/false based on this
    return true;
  }

  /**
   * @brief Write E6AXIS to KRC.
   *
   * @param write_to KRL E6AXIS variable name
   * @param joint_command Joint values, in order from A1 - A6, E1 - E6
   * @param joints Number of joint values to send
   *
   * @return Currently always returns true. Inteded to return true\false depning on succesfull write
   */
  bool writeE6AXIS(const std::string* write_to, const double* joint_command, const std::size_t joints)
  {
    std::string dummy;
    return writeE6AXIS(write_to, joint_command, joints, &dummy);
  }

  /**
   * @brief Write E6AXIS to KRC.
   *
   * @param write_to KRL E6AXIS variable name
   * @param joint_command Joint values, in order from A1 - A6, E1 - E6
   *
   * @return Currently always returns true. Inteded to return true\false depning on succesfull write
   */
  bool writeE6AXIS(const std::string* write_to, const double joint_command[12])
  {
    return writeE6AXIS(write_to, joint_command, 12);
  }

  /**
   * @brief Write E6AXIS to KRC.
   *
   * @param write_to KRL E6AXIS variable name
   * @param joint_command Map of values with A1 - E6 as keys
   * @param joints Number of joints to write
   *
   * @return Currently always return true. Intended to return true\false depending on succesfull write
   */
  bool writeE6AXIS(const std::string* write_to, std::map<std::string, double>& joint_command, std::size_t joints)
  {
    double joint_cmd[12];

    joint_cmd[0] = joint_command["A1"];
    joint_cmd[1] = joint_command["A2"];
    joint_cmd[2] = joint_command["A3"];
    joint_cmd[3] = joint_command["A4"];
    joint_cmd[4] = joint_command["A5"];
    joint_cmd[5] = joint_command["A6"];

    joint_cmd[6] = joint_command["E1"];
    joint_cmd[7] = joint_command["E2"];
    joint_cmd[8] = joint_command["E3"];
    joint_cmd[9] = joint_command["E4"];
    joint_cmd[10] = joint_command["E5"];
    joint_cmd[11] = joint_command["E6"];

    return writeE6AXIS(write_to, joint_cmd, joints);
  }

  /**
   * @brief Read KRL E6Axis variable
   *
   * @param read_from KRL E6Axis variable to read from
   * @param joint_pos Store joint values
   *
   * @return Returns false if read fails
   */
  bool readE6AXIS(const std::string* read_from, std::map<std::string, double>& joint_pos)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());

    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    if (reply.size() == 0)
    {
      return false;
    }

    std::string act_pos(reply.begin(), reply.end());

    // Check that we got atleast one ,
    if (act_pos.find(",") == std::string::npos)
    {
      return false;
    }

    // remove var name and trailing }
    act_pos = act_pos.substr(act_pos.find(": ") + 1);
    act_pos.pop_back();
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

    boost::char_separator<char> sep(", ");
    tokenizer tokens(act_pos, sep);
    tokenizer::iterator tok_var = tokens.begin();
    tokenizer::iterator tok_value;

    for (; tok_var != tokens.end(); std::advance(tok_var, 2))
    {
      tok_value = tok_var;
      tok_value++;
      try
      {
        joint_pos[*tok_var] = std::stod(*tok_value);
      }
      catch (const std::invalid_argument& e)
      {
        return false;
      }
    }

    return true;
  }

  /**
   * @brief Read KRL E6Axis variable
   *
   * @param read_from KRL E6Axis variable to read from
   * @param joint_pos[12] Store values to array
   *
   * @return Returns false if read fails
   */
  bool readE6AXIS(const std::string* read_from, double joint_pos[12])
  {
    std::map<std::string, double> joint_pos_map;
    bool ret = readE6AXIS(read_from, joint_pos_map);
    if (!ret)
    {
      return false;
    }

    joint_pos[0] = joint_pos_map["A1"];
    joint_pos[1] = joint_pos_map["A2"];
    joint_pos[2] = joint_pos_map["A3"];
    joint_pos[3] = joint_pos_map["A4"];
    joint_pos[4] = joint_pos_map["A5"];
    joint_pos[5] = joint_pos_map["A6"];

    joint_pos[6] = joint_pos_map["E1"];
    joint_pos[7] = joint_pos_map["E2"];
    joint_pos[8] = joint_pos_map["E3"];
    joint_pos[9] = joint_pos_map["E4"];
    joint_pos[10] = joint_pos_map["E5"];
    joint_pos[11] = joint_pos_map["E6"];

    return true;
  }

  /**
   * @brief Read a KRL struct
   *
   * @param read_from KRL variable name to read from
   * @param output Map to store return value to
   *
   * @return Returns false if read fails
   */
  bool readStruct(const std::string* read_from, std::map<std::string, std::string>& output)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());

    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    if (reply.size() == 0)
    {
      return false;
    }

    std::string act_pos(reply.begin(), reply.end());

    // remove var name and trailing }
    act_pos = act_pos.substr(act_pos.find(": ") + 1);
    act_pos.pop_back();

    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
    boost::char_separator<char> sep(", ");
    tokenizer tokens(act_pos, sep);
    tokenizer::iterator tok_var = tokens.begin();
    tokenizer::iterator tok_value;
    ;
    for (; tok_var != tokens.end(); std::advance(tok_var, 2))
    {
      tok_value = tok_var;
      tok_value++;
      try
      {
        output[*tok_var] = *tok_value;
      }
      catch (const std::invalid_argument& e)
      {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Reads boolean variable from KRC
   *
   * @param read_from Variable to read from
   * @param output Store value to this variable
   *
   * @return Returns false on error
   */
  bool readBool(const std::string* read_from, bool& output)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());
    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    if (reply.size() == 0)
    {
      return false;
    }

    std::string value(reply.begin(), reply.end());

    if (value == "TRUE")
    {
      output = true;
    }
    else if (value == "FALSE")
    {
      output = false;
    }
    else
    {
      // We do not like this output. Return false
      return false;
    }

    return true;
  }

  /**
   * @brief Write bolean value to KRC
   *
   * @param write_to Variable to write to
   * @param value Value to set
   *
   * @return Returns false on error $TODO: Fix this
   */
  bool writeBool(const std::string* write_to, const bool* value)
  {
    std::string out;
    out = *value ? "TRUE" : "FALSE";

    std::vector<unsigned char> out_vector(out.begin(), out.end());
    std::vector<unsigned char> var(write_to->begin(), write_to->end());
    std::vector<unsigned char> formated_out = this->formatWriteMsg(var, out_vector);
    std::vector<unsigned char> reply = this->sendMsg(formated_out);
    // TODO: Check reply from server and return true/false based on this
    return true;
  }

  /**
   * @brief Reads REAL (double) variable from KRC
   *
   * @param read_from Variable to read from
   * @param output Store value to this variable
   *
   * @return Returns false on error
   */
  bool readReal(const std::string* read_from, double& output)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());
    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    std::string value(reply.begin(), reply.end());
    try
      {
	output = std::stod(value);
      }
    catch (const std::invalid_argument& e)
      {
	return false;
      }
    return true;
  }

  /**
   * @brief Write REAL (double )value to KRC
   *
   * @param write_to Variable to write to
   * @param value Value to set
   *
   * @return Returns false on error $TODO: Fix this
   */
  bool writeReal(const std::string* write_to, const double* value)
  {
    std::string out = std::to_string(*value);

    std::vector<unsigned char> out_vector(out.begin(), out.end());
    std::vector<unsigned char> var(write_to->begin(), write_to->end());
    std::vector<unsigned char> formated_out = this->formatWriteMsg(var, out_vector);
    std::vector<unsigned char> reply = this->sendMsg(formated_out);
    // TODO: Check reply from server and return true/false based on this
    return true;
  }

  /**
   * @brief Reads INT variable from KRC
   *
   * @param read_from Variable to read from
   * @param output Store value to this variable
   *
   * @return Returns false on error
   */
  bool readInt(const std::string* read_from, int& output)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());

    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    std::string svalue(reply.begin(), reply.end());
    try
      {
	output = std::stoi(svalue);
      }
    catch (const std::invalid_argument& e)
      {
	return false;
      }
    return true;
  }

  /**
   * @brief Write INT value to KRC
   *
   * @param write_to Variable to write to
   * @param value Value to set
   *
   * @return Returns false on error $TODO: Fix this
   */
  bool writeInt(const std::string* write_to, const int* value)
  {
    std::string out = std::to_string(*value);

    std::vector<unsigned char> out_vector(out.begin(), out.end());
    std::vector<unsigned char> var(write_to->begin(), write_to->end());
    std::vector<unsigned char> formated_out = this->formatWriteMsg(var, out_vector);
    std::vector<unsigned char> reply = this->sendMsg(formated_out);
    // TODO: Check reply from server and return true/false based on this
    return true;
  }
  
  
  /**
   * @brief Write KRL Frame to KRC.
   *
   * @param write_to KRL Frame variable name
   * @param position_command Cartesian position, in order X, Y, Z
   * @param orientation_command Roll, pitch, yaw, using ZYX convention
   * @param out Formatted string sent to the KRC is stored in this variable
   *
   * @return Currently always returns true. Intended to return true/false depending on successful write.
   */
  bool writeFrame(const std::string* write_to, const double* position_command, const double* orientation_command, std::string* out)
  {
    *out = "{FRAME: ";
    *out += " X" + std::to_string(position_command[0]) + ",";
    *out += " Y" + std::to_string(position_command[1]) + ",";
    *out += " Z" + std::to_string(position_command[2]) + ",";
    *out += " A" + std::to_string(orientation_command[2]) + ",";
    *out += " B" + std::to_string(orientation_command[1]) + ",";
    *out += " C" + std::to_string(orientation_command[0]) + ",";
    out->back() = '}';

    std::vector<unsigned char> out_vector(out->begin(), out->end());
    std::vector<unsigned char> var(write_to->begin(), write_to->end());
    std::vector<unsigned char> formated_out = this->formatWriteMsg(var, out_vector);
    std::vector<unsigned char> reply = this->sendMsg(formated_out);
    return true;
  }

  /**
   * @brief Write KRL Frame to KRC.
   *
   * @param write_to KRL Frame variable name
   * @param position_command Cartesian position, in order X, Y, Z
   * @param orientation_command Roll, pitch, yaw, using ZYX convention
   *
   * @return Currently always returns true. Intended to return true/false depending on successful write.
   */
  bool writeFrame(const std::string* write_to, const double* position_command, const double* orientation_command)
  {
    std::string dummy;
    return writeFrame(write_to, position_command, orientation_command, &dummy);
  }

  /**
   * @brief Write KRL Frame to KRC.
   *
   * @param write_to KRL Frame variable name
   * @param frame_command Map of values with X,Y,Z, ROLL, PITCH, YAW as keys
   *
   * @return Currently always returns true. Intended to return true/false depending on successful write.
   */
  bool writeFrame(const std::string* write_to, std::map<std::string, double>& frame_command)
  {
    double position_command[3];
    position_command[0] = frame_command["X"];
    position_command[1] = frame_command["Y"];
    position_command[2] = frame_command["Z"];
    double orientation_command[3];
    orientation_command[0] = frame_command["ROLL"];
    orientation_command[1] = frame_command["PITCH"];
    orientation_command[2] = frame_command["YAW"];
    return writeFrame(write_to, position_command, orientation_command);
  }

  /**
   * @brief Read KRL Frame to KRC.
   *
   * @param read_from KRL Frame variable to read from
   * @param Frame Map of values with X,Y,Z,ROLL,PITCH, YAW as keys that we write to
   *
   * @return Returns false if read fails.
   */
  bool readFrame(const std::string* read_from, std::map<std::string, double>& frame)
  {
    std::vector<unsigned char> var(read_from->begin(), read_from->end());
    std::vector<unsigned char> formated_read = this->formatReadMsg(var);
    std::vector<unsigned char> reply = this->sendMsg(formated_read);
    if (reply.size() == 0)
      {
	return false;
      }

    std::string act_frame(reply.begin(), reply.end());

    // Check that we got at least one ,
    if (act_frame.find(",") == std::string::npos)
      {
	return false;
      }

    // Remove var name and trailing }
    act_frame = act_frame.substr(act_frame.find(": ")+1);
    act_frame.pop_back();
    typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

    boost::char_separator<char> sep(", ");
    tokenizer tokens(act_frame, sep);
    tokenizer::iterator tok_var = tokens.begin();
    tokenizer::iterator tok_value;

    for (; tok_var != tokens.end(); std::advance(tok_var, 2))
      {
	tok_value = tok_var;
	tok_value++;
	try
	  {
	    frame[*tok_var] = std::stod(*tok_value);
	  }
	catch (const std::invalid_argument& e)
	  {
	    return false;
	  }
      }
    return true;
  }

  /**
   * @brief Read KRL Frame to KRC.
   *
   * @param read_from KRL Frame variable to read from
   * @param position[3] XYZ cartesian position
   * @param orientation[3] roll, pitch, yaw in XYZ convention
   *
   * @return Returns false if read fails.
   */
  bool readFrame(const std::string* read_from, double position[3], double orientation[3])
  {
    std::map<std::string, double> frame_map;
    bool ret = readFrame(read_from, frame_map);
    if (!ret)
      {
	return false;
      }
    position[0] = frame_map["X"];
    position[1] = frame_map["Y"];
    position[2] = frame_map["Z"];
    orientation[0] = frame_map["ROLL"];
    orientation[1] = frame_map["PITCH"];
    orientation[2] = frame_map["YAW"];

    return true;
  }
};  // class

#endif
