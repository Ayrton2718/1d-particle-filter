#include "lc_map/lc_map.hpp"

#include <msgpack.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>


#ifndef LCMCL_MAP_DIRECTORY
#error LCMCL_MAP_DIRECTORY dose not set.
#endif /*LCMCL_MAP_DIRECTORY*/

using namespace msgpack::MSGPACK_DEFAULT_API_NS;

namespace lc
{


void Map::import_from_file(zone_t zone)
{
    std::string file_name = (std::string)LCMCL_MAP_DIRECTORY + "/map.mpac";
    
    if(_info)
        TAGGER(_info.get(), "Map load start (%s)", file_name.c_str());

    FILE* fp = fopen(file_name.c_str(), "r");
    if(fseek(fp, 0, SEEK_END) != 0)
    {
        if(_error)
            TAGGER(_error.get(), "Failed to fseek");
    }

    size_t len = ftell(fp);
    if(fseek(fp, 0L, SEEK_SET) != 0)
    {
        if(_error)
            TAGGER(_error.get(), "Failed to fseek");
    }

    char* buff = (char*)malloc(len);
    if(fread(buff, 1, len, fp) < len)
    {
        if(_error)
            TAGGER(_error.get(), "Failed to fread");
    }

    this->import_from_raw(len, buff, zone);

    free(buff);
    fclose(fp);
}


void Map::import_from_raw(size_t len, const char* bin, zone_t zone)
{
    msgpack::object_handle oh = msgpack::unpack(bin, len);
    msgpack::object deserialized = oh.get();
    if(deserialized.type == msgpack::type::object_type::MAP)
    {
        std::map<std::string, msgpack::object> root;

        if(root.find("max_pos") != root.end())
        {
            root["max_pos"].convert(this->_max_pos);
        }else{
            this->_max_pos[0] = 0;
            this->_max_pos[1] = 0;

            if(_error)
                TAGGER(_error.get(), "Unknown max_pos!");
        }
                deserialized.convert(root);

        if(root.find("min_pos") != root.end())
        {
            root["min_pos"].convert(this->_min_pos);
        }else{
            this->_min_pos[0] = 0;
            this->_min_pos[1] = 0;

            if(_error)
                TAGGER(_error.get(), "Unknown min_pos!");
        }

        if(root.find("obs_types") != root.end())
        {
            std::vector<std::tuple<std::string, uint32_t>> var;
            root["obs_types"].convert(var);

            for(auto it = var.begin(); it != var.end(); it++)
            {
                line_type_t lt;
                lt.name = std::get<0>(*it);
                lt.r = (std::get<1>(*it) & 0x00FF0000) >> 16;
                lt.g = (std::get<1>(*it) & 0x0000FF00) >> 8;
                lt.b = (std::get<1>(*it) & 0x000000FF);
                this->_obs_types.push_back(lt);
            }
        }else{
            if(_error)
                TAGGER(_error.get(), "Unknown obs_types!");
        }

        _obs_lines.resize(_obs_types.size());
        _obs_circles.resize(_obs_types.size());

        if(root.find("obs_lines") != root.end())
        {
            std::vector<std::vector<std::tuple<float, float, float, float>>> var;
            root["obs_lines"].convert(var);

            size_t type_i = 0;
            for(auto list_it = var.begin(); list_it !=var.end(); list_it++, type_i++)
            {
                for(auto line_it = list_it->begin(); line_it != list_it->end(); line_it++)
                {
                    line_t l;
                    l.x0 = std::get<0>(*line_it);
                    l.y0 = std::get<1>(*line_it);
                    l.vx = std::get<2>(*line_it);
                    l.vy = std::get<3>(*line_it);

                    std::array<float, 4> l_pos = {l.x0, l.y0, l.x0+l.vx, l.y0+l.vy};
                    if(zone == zone_t::RED){
                        if(0 < l_pos[1] || 0 < l_pos[3]){
                            this->_obs_lines[type_i].push_back(l);
                        }
                    }else if(zone == zone_t::BLUE){
                        if(l_pos[1] < 0 || l_pos[3] < 0){
                            this->_obs_lines[type_i].push_back(l);
                        }
                    }else{
                        this->_obs_lines[type_i].push_back(l);
                    }
                }
            }
        }else{
            if(_error)
                TAGGER(_error.get(), "Unknown obs_lines!");
        }

        if(root.find("obs_circles") != root.end())
        {
            std::vector<std::vector<std::tuple<float, float, float>>> var;
            root["obs_circles"].convert(var);

            size_t type_i = 0;
            for(auto list_it = var.begin(); list_it !=var.end(); list_it++, type_i++)
            {
                for(auto circle_it = list_it->begin(); circle_it != list_it->end(); circle_it++)
                {
                    circle_t c;
                    c.x0 = std::get<0>(*circle_it);
                    c.y0 = std::get<1>(*circle_it);
                    c.r = std::get<2>(*circle_it);

                    if(zone == zone_t::RED){
                        if(0 < c.y0){
                            this->_obs_circles[type_i].push_back(c);
                        }
                    }else if(zone == zone_t::BLUE){
                        if(c.y0 < 0){
                            this->_obs_circles[type_i].push_back(c);
                        }
                    }else{
                        this->_obs_circles[type_i].push_back(c);
                    }
                }
            }
        }else{
            if(_error)
                TAGGER(_error.get(), "Unknown obs_lines!");
        }
    }
}

}