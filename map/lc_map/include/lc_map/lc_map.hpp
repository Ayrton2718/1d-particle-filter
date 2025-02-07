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
            // ※以下のローカル変数展開により、メンバ変数への間接参照を減らす
            const double ox = _origin_x;
            const double oy = _origin_y;
            const double s = _scale;
            const int grid_x = _size_x;
            const int grid_y = _size_y;
            const cell_t *cells_ptr = _cells.data();

            // ワールド座標 → マップインデックス（inline展開）
            auto worldToMap = [=](double coord, double origin, int grid_size) -> int
            {
                return static_cast<int>(std::floor((coord - origin) / s + 0.5)) + grid_size / 2;
            };

            // 初期位置
            int x0 = worldToMap(pos.x, ox, grid_x);
            int y0 = worldToMap(pos.y, oy, grid_y);
            // 終端位置（最大レンジ方向）
            int x1 = worldToMap(pos.x + max_range * std::cos(pos.rad), ox, grid_x);
            int y1 = worldToMap(pos.y + max_range * std::sin(pos.rad), oy, grid_y);

            // steep判定
            bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));
            int x0_local = x0, y0_local = y0, x1_local = x1, y1_local = y1;
            if (steep)
            {
                std::swap(x0_local, y0_local);
                std::swap(x1_local, y1_local);
            }

            const int dx = std::abs(x1_local - x0_local);
            const int dy = std::abs(y1_local - y0_local);
            int error = 0;
            const int delta_err = dy;

            int x = x0_local;
            int y = y0_local;
            const int x_step = (x0_local < x1_local) ? 1 : -1;
            const int y_step = (y0_local < y1_local) ? 1 : -1;

            // ラムダ: steep/non-steepでセルのチェック（インデックス計算を展開）
            auto cell_is_obstacle = [&](int a, int b) -> bool
            {
                // steep時は a: y, b: x、それ以外は a: x, b: y としてチェック
                if (!steep)
                {
                    if (a < 0 || a >= grid_x || b < 0 || b >= grid_y)
                        return true;
                    return (cells_ptr[a + b * grid_x].occ_state > -1);
                }
                else
                {
                    if (a < 0 || a >= grid_x || b < 0 || b >= grid_y)
                        return true;
                    return (cells_ptr[a + b * grid_x].occ_state > -1);
                }
            };

            // 最初のセルのチェック
            if (steep)
            {
                if (cell_is_obstacle(y, x))
                {
                    laser_t hit;
                    hit.hit_type = 1;
                    int dx_local = x - x0_local;
                    int dy_local = y - y0_local;
                    hit.range = std::sqrt(dx_local * dx_local + dy_local * dy_local) * s;
                    return hit;
                }
            }
            else
            {
                if (cell_is_obstacle(x, y))
                {
                    laser_t hit;
                    hit.hit_type = 1;
                    int dx_local = x - x0_local;
                    int dy_local = y - y0_local;
                    hit.range = std::sqrt(dx_local * dx_local + dy_local * dy_local) * s;
                    return hit;
                }
            }

            // Bresenhamループ
            while (x != (x1_local + x_step))
            {
                x += x_step;
                error += delta_err;
                if (2 * error >= dx)
                {
                    y += y_step;
                    error -= dx;
                }

                if (steep)
                {
                    if (cell_is_obstacle(y, x))
                    {
                        laser_t hit;
                        hit.hit_type = 1;
                        int dx_local = x - x0_local;
                        int dy_local = y - y0_local;
                        hit.range = std::sqrt(dx_local * dx_local + dy_local * dy_local) * s;
                        return hit;
                    }
                }
                else
                {
                    if (cell_is_obstacle(x, y))
                    {
                        laser_t hit;
                        hit.hit_type = 1;
                        int dx_local = x - x0_local;
                        int dy_local = y - y0_local;
                        hit.range = std::sqrt(dx_local * dx_local + dy_local * dy_local) * s;
                        return hit;
                    }
                }
            }

            // 障害物がなかった場合
            laser_t hit;
            hit.hit_type = -1;
            hit.range = max_range;
            return hit;
        }
    };

} // namespace lc
