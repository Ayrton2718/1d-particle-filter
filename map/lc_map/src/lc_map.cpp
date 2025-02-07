#include "lc_map/lc_map.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <fstream>  // Include for std::ifstream
#include <string>   // Include if you are using std::string
#include <iostream> // Optional, for debugging or output



#ifndef LCMCL_MAP_DIRECTORY
#error LCMCL_MAP_DIRECTORY dose not set.
#endif /*LCMCL_MAP_DIRECTORY*/


namespace lc
{

static void skipComments(std::istream &stream);

void Map::import_yaml(void)
{
    std::string yaml_file = (std::string)LCMCL_MAP_DIRECTORY + "/map.yaml";

    std::ifstream file(yaml_file);
    if (!file) {
        throw std::runtime_error("Unable to open map config file: " + yaml_file);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("resolution:") == 0) {
            _scale = std::stod(line.substr(line.find_first_of(':') + 1));
        } else if (line.find("origin:") == 0) {
            auto start = line.find('[') + 1;
            auto end = line.find(']');
            auto values = line.substr(start, end - start);
            sscanf(values.c_str(), "%lf, %lf, %lf", &_origin_x, &_origin_y, &_origin_theta);
        } else if (line.find("occupied_thresh:") == 0) {
            _occupied_thresh = std::stod(line.substr(line.find_first_of(':') + 1));
        } else if (line.find("free_thresh:") == 0) {
            _free_thresh = std::stod(line.substr(line.find_first_of(':') + 1));
        }
    }
}

void Map::import_pgm(void)
{
    std::string pgm_file = (std::string)LCMCL_MAP_DIRECTORY + "/map.pgm";

    std::ifstream file(pgm_file, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Unable to open file: " + std::string(strerror(errno)));
    }

    std::string magic;
    file >> magic;
    if (magic != "P5") {
        throw std::runtime_error("Incorrect image format; must be PGM/binary");
    }

    skipComments(file);
    int width, height;
    file >> width >> height;
    skipComments(file);

    int depth;
    file >> depth;
    file.ignore(1);

    _size_x = width;
    _size_y = height;
    _origin_x = _origin_x + (_size_x / 2) * _scale;
    _origin_y = _origin_y + (_size_y / 2) * _scale;

    allocate(width, height);

    for (int j = height - 1; j >= 0; --j) {
        for (int i = 0; i < width; ++i) {
            unsigned char ch;
            file.read(reinterpret_cast<char*>(&ch), sizeof(ch));
            double occ = static_cast<double>(ch) / depth;

            if (isValid(i, j)) {
                if (occ >= _occupied_thresh) {
                    getCell(i, j).occ_state = -1;
                } else if (occ <= _free_thresh) {
                    getCell(i, j).occ_state = 1;
                } else {
                    getCell(i, j).occ_state = 0;
                }
            }
        }
    }

    TAGGER(_info.get(), "Map loaded: %d x %d", width, height);
}

// Helper function to skip comment lines and whitespace
static void skipComments(std::istream &stream) {
    // Skip any whitespace or comment lines (that start with '#')
    while (std::isspace(stream.peek()) || stream.peek() == '#') {
        if (stream.peek() == '#') {
            // Skip the entire comment line
            stream.ignore(1024, '\n');
        } else {
            // Skip a single whitespace character
            stream.get();
        }
    }
}


}