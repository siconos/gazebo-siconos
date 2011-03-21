/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef CONNECTION_HH
#define CONNECTION_HH

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <iomanip>

#include <google/protobuf/message.h>

#define HEADER_LENGTH 8

namespace gazebo
{
  namespace transport
  {
    class Connection
    {
      public: Connection(boost::asio::io_service &io_service);
      public: virtual ~Connection();

      public: boost::asio::ip::tcp::socket &GetSocket();

      public: void Connect(const std::string &host, const std::string &service);

      public: template<typename Handler>
              void Write(const google::protobuf::Message &msg, Handler handler)
              {
                std::string out;
                msg.SerializeToString(&out);
                this->Write(out, handler);
              }

      public: template<typename Handler>
              void Write(const std::string buffer, Handler handler)
              {
                std::ostringstream header_stream;
                header_stream << std::setw(HEADER_LENGTH) 
                              << std::hex << buffer.size();

                if (!header_stream || header_stream.str().size() != HEADER_LENGTH)
                {
                  //Something went wrong, inform the caller
                  boost::system::error_code error(boost::asio::error::invalid_argument);
                  this->socket.io_service().post(boost::bind(handler,error));
                }

                this->outbound_header = header_stream.str();
                this->outbound_data = buffer;

                // Write the serialized data to the socket. We use
                // "gather-write" to send both the head and the data in
                // a single write operation
                std::vector<boost::asio::const_buffer> buffers;
                buffers.push_back(boost::asio::buffer(this->outbound_header));
                buffers.push_back(boost::asio::buffer(this->outbound_data));
                boost::asio::async_write( this->socket, buffers, handler );
              }

      public: template<typename Handler>
              void Read(Handler handler)
              {
                void (Connection::*f)(const boost::system::error_code &,
                    boost::tuple<Handler>) = &Connection::OnReadHeader<Handler>;

                boost::asio::async_read(this->socket,
                    boost::asio::buffer(this->inbound_header),
                    boost::bind(f, this, boost::asio::placeholders::error,
                                boost::make_tuple(handler)) );
              }

      // Handle a completed read of a message header. The handler is passed
      // using a tuple since boost::bind seems to have trouble binding
      // a function object created using boost::bind as a parameter
      public: template<typename Handler>
              void OnReadHeader(const boost::system::error_code &e,
                                boost::tuple<Handler> handler)
              {
                std::cout << "OnReadHeader\n";
                if (e)
                {
                  std::cout << "An error occrured here\n";
                  // Pass the error to the handler
                  //boost::get<0>(handler)(e);
                }
                else
                {
                  // Determine the length of the serialized data
                  std::istringstream is(std::string(this->inbound_header, HEADER_LENGTH));
                  std::size_t inbound_data_size = 0;
                  if (!(is >> std::hex >> inbound_data_size))
                  {
                    // Header doesn't seem to be valid. Inform the caller
                    boost::system::error_code error(boost::asio::error::invalid_argument);
                    std::cerr << "New Error:" << error.message() << "\n";
                    //boost::get<0>(handler)(error);
                  }


                  std::cout << "Inbound size[" << inbound_data_size << "]\n";
                  // Start the asynchronous call to receive data
                  this->inbound_data.resize(inbound_data_size);

                  void (Connection::*f)(const boost::system::error_code &e,
                     boost::tuple<Handler>) = &Connection::OnReadData<Handler>;

                  std::cout << "Async read stuff\n";
                  boost::asio::async_read( this->socket, 
                      boost::asio::buffer(this->inbound_data), 
                      boost::bind(f, this, boost::asio::placeholders::error, 
                                  handler) );
                }
              }

     public: template<typename Handler>
              void OnReadData(const boost::system::error_code &e,
                              boost::tuple<Handler> handler)
               {
                 std::cout << "OnReadData\n";
                 if (e)
                 {
                   std::cerr << "Error:" << e.message() << std::endl;

                   // TODO: Pass error to handler
                   //boost::get<0>(handler)(e);
                 }
                 else
                 {
                   // Extract the data structure from the data
                   /*try
                   {
                     std::string archive_data(&this->inbound_data[0], this->inbound_data.size());
                     std::istringstream archive_stream(archive_data);
                     boost::archive::text_iarchive archive(archive_stream);
                     archive >> t;
                   }
                   catch (std::exception &e)
                   {
                     // Unable to decode data
                     boost::system::error_code error(boost::asio::error::invalid_argument);
                     std::cerr << "Error as well\n";
                     return;
                   }
                   */

                   //helper->OnRead(this->inbound_data);
                   
                   // Inform caller that data has been received
                   std::string data(&this->inbound_data[0], 
                                    this->inbound_data.size());
                   boost::get<0>(handler)(data);
                 }
               }


      private: boost::asio::ip::tcp::socket socket;
      private: std::string outbound_header;
      private: std::string outbound_data;

      private: char inbound_header[HEADER_LENGTH];
      private: std::vector<char> inbound_data;
    };

    typedef boost::shared_ptr<Connection> ConnectionPtr;
  }
}

#endif
