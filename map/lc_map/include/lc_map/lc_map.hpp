#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <vector>
#include <cmath>
#include <float.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <blackbox/blackbox.hpp>
#include "lc_type.hpp"
#include "lc_tf.hpp"
#include <Eigen/Geometry>

namespace lc
{

    class Map
    {
    public:
        typedef struct
        {
            int16_t hit_type;
            float range;
        } laser_t;

    private:
        typedef struct
        {
            int occ_state;   // Occupancy state (-1 = free, 0 = unknown, +1 = occupied)
            double occ_dist; // Distance to the nearest occupied cell
        } cell_t;

    private:
        rclcpp::Node *_node;
        std::unique_ptr<blackbox::Logger> _error;
        std::unique_ptr<blackbox::Logger> _info;

        // マップパラメータ
        double _origin_x, _origin_y, _origin_theta; // マップ原点
        double _scale;                              // 解像度 (meters per cell)
        int _size_x, _size_y;                       // マップサイズ（セル数）
        std::vector<cell_t> _cells;                 // グリッドデータ
        double _max_occ_dist;                       // 障害物考慮の最大距離
        double _occupied_thresh;                    // 占有セルの閾値
        double _free_thresh;                        // 自由セルの閾値

        // YAML/PGMからマップデータをインポートする（実装は別途）
        void import_yaml(void);
        void import_pgm(void);

        // マップ領域の確保
        void allocate(int width, int height)
        {
            _size_x = width;
            _size_y = height;
            _cells.resize(width * height);
        }

        // 指定セルの参照を返す
        cell_t &getCell(int i, int j)
        {
            if (!isValid(i, j))
            {
                throw std::out_of_range("Invalid cell coordinates");
            }
            return _cells[i + j * _size_x];
        }

        // 指定セル座標が有効かどうか判定
        bool isValid(int i, int j) const
        {
            return (i >= 0 && i < _size_x && j >= 0 && j < _size_y);
        }

        // マクロの代替としてインライン関数で変換処理を実装

        // マップインデックス (i) からワールド座標 x を求める
        double mapWxGX(int i) const
        {
            return _origin_x + (i - _size_x / 2) * _scale;
        }

        // マップインデックス (j) からワールド座標 y を求める
        double mapWyGY(int j) const
        {
            return _origin_y + (j - _size_y / 2) * _scale;
        }

        // ワールド座標 x からマップインデックスを求める
        int mapGxWX(double x) const
        {
            return static_cast<int>(std::floor((x - _origin_x) / _scale + 0.5)) + _size_x / 2;
        }

        // ワールド座標 y からマップインデックスを求める
        int mapGyWY(double y) const
        {
            return static_cast<int>(std::floor((y - _origin_y) / _scale + 0.5)) + _size_y / 2;
        }

        // 指定したマップインデックスが有効か判定
        bool mapValid(int i, int j) const
        {
            return (i >= 0 && i < _size_x && j >= 0 && j < _size_y);
        }

        // マップインデックス (i, j) に対するセル配列上のインデックスを返す
        int mapIndex(int i, int j) const
        {
            return i + j * _size_x;
        }

    public:
        // コンストラクタ (rclcpp::Node* を用いた初期化)
        explicit Map(blackbox::BlackBoxNode* node)
            : _node(node)
        {
            _origin_x = 0,
            _origin_y = 0,
            _origin_theta = 0,
            _scale = 0;
            _size_x = 0;
            _size_y = 0;
            _max_occ_dist = 0;
            _occupied_thresh = 0.65;
            _free_thresh = 0.196;
            
            _error = std::make_unique<blackbox::Logger>();
            _info = std::make_unique<blackbox::Logger>();
            _error->init(node, blackbox::ERR, "lc_map");
            _info->init(node, blackbox::INFO, "lc_map");

            import_yaml();
            import_pgm();
        }

        laser_t calculate_range(pos_t pos, float max_range)
        {
            int x0, x1, y0, y1;
            int x, y;
            int x_step, y_step;
            bool steep;
            int tmp;
            int delta_x, delta_y, error, delta_err;

            x0 = mapGxWX(pos.x);
            y0 = mapGyWY(pos.y);
            x1 = mapGxWX(pos.x + max_range * std::cos(pos.rad));
            y1 = mapGyWY(pos.y + max_range * std::sin(pos.rad));

            steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

            if (steep)
            {
                tmp = x0;
                x0 = y0;
                y0 = tmp;
                tmp = x1;
                x1 = y1;
                y1 = tmp;
            }

            delta_x = std::abs(x1 - x0);
            delta_y = std::abs(y1 - y0);
            error = 0;
            delta_err = delta_y;

            x = x0;
            y = y0;
            x_step = (x0 < x1) ? 1 : -1;
            y_step = (y0 < y1) ? 1 : -1;

            if (steep)
            {
                if (!mapValid(y, x) || _cells[mapIndex(y, x)].occ_state > -1)
                {
                    laser_t hit;
                    hit.hit_type = 1;
                    hit.range = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _scale;
                    return hit;
                }
            }
            else
            {
                if (!mapValid(x, y) || _cells[mapIndex(x, y)].occ_state > -1)
                {
                    laser_t hit;
                    hit.hit_type = 1;
                    hit.range = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _scale;
                    return hit;
                }
            }

            while (x != (x1 + x_step))
            {
                x += x_step;
                error += delta_err;
                if (2 * error >= delta_x)
                {
                    y += y_step;
                    error -= delta_x;
                }

                if (steep)
                {
                    if (!mapValid(y, x) || _cells[mapIndex(y, x)].occ_state > -1)
                    {
                        laser_t hit;
                        hit.hit_type = 1;
                        hit.range = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _scale;
                        return hit;
                    }
                }
                else
                {
                    if (!mapValid(x, y) || _cells[mapIndex(x, y)].occ_state > -1)
                    {
                        laser_t hit;
                        hit.hit_type = 1;
                        hit.range = std::sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _scale;
                        return hit;
                    }
                }
            }

            laser_t hit;
            hit.hit_type = -1;
            hit.range = max_range;
            return hit;
        }
    };

} // namespace lc
