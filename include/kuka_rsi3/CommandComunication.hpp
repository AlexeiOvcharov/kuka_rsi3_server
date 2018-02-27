#ifndef CLIENT
#define CLIENT

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define YELLOW  "\033[33m"      /* Yellow */
#define GREEN   "\033[32m"      /* Green */
#define RESET   "\033[0m"

using namespace boost::asio;

class CommandClient 
{
    public:
        CommandClient(std::string host, short int port, io_service & io_service)
            : socket(io_service), serverEP(ip::address::from_string(host), port)
        {}
        ~CommandClient()
        {}

        void connect()
        {
            bool retry = true, viewMessage = true;
            int waitTime = 1; // seconds
            boost::system::error_code error;

            socket.close();

            while (retry) {
                socket.connect(serverEP, error);
                if (error && viewMessage) {
                    std::cout << YELLOW << "Waiting for server connection." << RESET << std::endl;
                    boost::this_thread::sleep_for(boost::chrono::seconds{waitTime});
                    viewMessage = false;
                } else if (error) {
                    boost::this_thread::sleep_for(boost::chrono::seconds{waitTime});
                } else
                    retry = false;
            }

            std::cout << "[TCP] Client connect to (" << serverEP.address() << ", " << serverEP.port() << ")" << std::endl;
        }

        void send(std::string & command)
        {
            try {
                boost::system::error_code error;
                write(socket, buffer(command), error);

                if (error)
                    throw boost::system::system_error(error);

            } catch (std::exception& e) {
                std::cerr << e.what() << std::endl;
            }

        }

        std::string read()
        {
            try {
                boost::system::error_code error;
                socket.read_some(buffer(msg), error);

                if (error)
                    throw boost::system::system_error(error);

            } catch (std::exception& e) {
                std::cerr << e.what() << std::endl;
            }

            return std::string(msg.data());
        }



    private:
        /// The tcp client socket
        ip::tcp::socket socket;

        /// Server endpoint (ip and port)
        ip::tcp::endpoint serverEP;

        /// Received message
        boost::array<char, 128> msg;

};

class CommandServer 
{
    public:
        CommandServer(std::string host, short int port, io_service & io_service)
            : socket(io_service), acceptor(io_service)
        {
            serverEP = ip::tcp::endpoint(ip::address::from_string(host), port);
            acceptor.open(serverEP.protocol());
            acceptor.bind(ip::tcp::endpoint(ip::address::from_string(host), port));
            acceptor.listen();
            std::cout << "[TCP] " << "Create socket at addr: (" << host << ", " << port << ")" << std::endl;
            running = false;
        }
        ~CommandServer()
        {}

        void accept() {
            acceptor.accept(socket);
            remoteEP = socket.remote_endpoint();
            std::cout << "[TCP] " << GREEN << "Client is connected: (" << remoteEP.address() << ", " << remoteEP.port() << ")" << RESET << std::endl;
            running = true;
        }

        void send(std::string & command)
        {
            try {
                boost::system::error_code error;
                write(socket, buffer(command), error);

                if (error)
                    throw boost::system::system_error(error);

            } catch (std::exception& e) {
                std::cerr << e.what() << std::endl;
            }

        }
        std::string read()
        {
            try {
                boost::system::error_code error;

                socket.read_some(buffer(msg));
                if (error)
                    throw boost::system::system_error(error);

            } catch (std::exception& e) {
                std::cerr << e.what() << std::endl;
                running = false;
                socket.close();
            }

            return std::string(msg.data());
        }

        bool is_ok() {
            return running;
        }


    private:
        /// The tcp client socket
        ip::tcp::socket socket;

        /// TCP acceptor
        ip::tcp::acceptor acceptor;

        /// Server endpoint (ip and port)
        ip::tcp::endpoint serverEP;

        /// Remote endpoint
        ip::tcp::endpoint remoteEP;

        /// Received message
        boost::array<char, 128> msg;

        /// True if socket is accepted
        bool running;
};
#endif
