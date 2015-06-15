/*************************************************************************
  > File Name: net_transfor.h
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 16 Jul 2014 02:12:45 PM CST
 ************************************************************************/
#ifndef NET_TRANSFOR_H_
#define  NET_TRANSFOR_H_

#include <iostream>
#include <boost/asio.hpp>
#include <vector>
#include <pcl/io/pcd_io.h>
using namespace std;
using boost::asio::ip::tcp;

class Net_transfor
{
    public:
        Net_transfor(int marker_):marker(marker_),
        endpoint(tcp::v4(),static_cast<unsigned int> (3556)),
        acceptor(io_service,endpoint),
        s(io_service),
        m1(true),
        m2(true),
        resolver(io_service),
        query(tcp::v4(),"192.168.0.168","3456"),
        iterator(resolver.resolve(query))
        {}

        bool send(pcl::PointCloud<pcl::PointXYZ>& cloud)
        {
            if(m2)
            {
                s.connect(*iterator);
                m2 = false;
            }
            int num = cloud.size();
            vector<short> points(num*3);
            const int conversion = 1000;
            for(size_t i =0;i<num;++i)
            {
                const pcl::PointXYZ& point = cloud.points[i];
                points[i*3+0] = static_cast<short>(point.x*conversion);
                points[i*3+1] = static_cast<short>(point.y*conversion);
                points[i*3+2] = static_cast<short>(point.z*conversion);
            }
            vector<int> num_of_pt(1);
            num_of_pt[0] = points.size();
            try{
                boost::asio::write(s,boost::asio::buffer(num_of_pt));
                boost::asio::write(s,boost::asio::buffer(points));
            }
            catch(boost::system::system_error const& e)
            {
                cout <<"warning:"<<e.what()<<endl;
            }
            cout <<"send over" <<endl;
        }

        bool receive(pcl::PointCloud<pcl::PointXYZ>& cloud)
        {
            if(m1)
            {
                acceptor.accept(s);
                m1 = false;
            }
            vector<int> num_of_pt(1);
            try
            {
                if(s.is_open()==false)
                    return false;
                boost::asio::read(s,boost::asio::buffer(num_of_pt),boost::asio::transfer_all());
            }
            catch(boost::system::system_error const& e)
            {
                cout <<"warning:"<<e.what()<<endl;
            }
            int num = num_of_pt[0];
            cout <<"num="<<num<<endl;
            if((num == -1) || (num == 0))
                return false;
            cloud.clear();
            vector<short> buf_pt(num);
            try
            {
                if(s.is_open()==false)
                    return false;
                boost::asio::read(s,boost::asio::buffer(buf_pt),boost::asio::transfer_all());
            }
            catch(boost::system::system_error const& e)
            {
                cout <<"warning:"<<e.what()<<endl;
            }
            cloud.clear();
            for(int i=0;i<num;i+=3)
            {
                pcl::PointXYZ temp;
                double x = static_cast<double>(buf_pt[i]);
                double y = static_cast<double>(buf_pt[i+1]);
                double z = static_cast<double>(buf_pt[i+2]);
                temp.x = x/1000;
                temp.y = y/1000;
                temp.z = z/1000;
                cloud.push_back(temp);
            }
            cout <<"received "<<cloud.size()<<"points\n";
            return true;
        }

        void close_receive()
        {
            s.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
            s.close();
        }

    private:
        int marker;
        boost::asio::io_service io_service;
        tcp::endpoint endpoint;
        tcp::acceptor acceptor;
        tcp::socket s;

        bool m1;//用于开启accept
        bool m2;//connect
        tcp::resolver resolver;
        tcp::resolver::query query;
        tcp::resolver::iterator iterator;

};

#endif
