#ifndef RAYTRACING_H
#define RAYTRACING_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>

template <typename PointT>
class RayTracing: public pcl::VoxelGridOcclusionEstimation<PointT> {
public:
    int rayTraceWrapper (int& out_state,
                         const Eigen::Vector4f& start,
                         const Eigen::Vector4f& end,
                         const float theta,
                         std::vector <Eigen::Vector3i>& out_ray)
    {
        Eigen::Vector3i start_ijk = this->getGridCoordinatesRound(start[0], start[1], start[2]);

        // estimate direction to target voxel
        Eigen::Vector4f direction = start - end;
        direction.normalize();

        // estimate entry point into the voxel grid
        float tmin = this->rayBoxIntersection (end, direction);

        if (tmin == -1) {
            PCL_ERROR ("The ray does not intersect with the bounding box \n");
            return -1;
        }

        // ray traversal
        out_state = rayTraversalNew (start_ijk, end, direction, tmin, theta, out_ray);

        return 0;
    }

private:
    int rayTraversalNew (const Eigen::Vector3i& target_voxel,
                         const Eigen::Vector4f& origin,
                         const Eigen::Vector4f& direction,
                         const float t_min,
                         const float theta,
                         std::vector <Eigen::Vector3i>& out_ray)
    {
        // coordinate of the boundary of the voxel grid
        Eigen::Vector4f start = origin;// + t_min * direction;

        // i,j,k coordinate of the voxel were the ray enters the voxel grid
        Eigen::Vector3i ijk = this->getGridCoordinatesRound (start[0], start[1], start[2]);

        // steps in which direction we have to travel in the voxel grid
        int step_x, step_y, step_z;

        // centroid coordinate of the entry voxel
        Eigen::Vector4f voxel_max = this->getCentroidCoordinate(ijk);

        if (direction[0] >= 0)
        {
            voxel_max[0] += this->leaf_size_[0] * 0.5f;
            step_x = 1;
        }
        else
        {
            voxel_max[0] -= this->leaf_size_[0] * 0.5f;
            step_x = -1;
        }
        if (direction[1] >= 0)
        {
            voxel_max[1] += this->leaf_size_[1] * 0.5f;
            step_y = 1;
        }
        else
        {
            voxel_max[1] -= this->leaf_size_[1] * 0.5f;
            step_y = -1;
        }
        if (direction[2] >= 0)
        {
            voxel_max[2] += this->leaf_size_[2] * 0.5f;
            step_z = 1;
        }
        else
        {
            voxel_max[2] -= this->leaf_size_[2] * 0.5f;
            step_z = -1;
        }

        float t_max_x = t_min + (voxel_max[0] - start[0]) / direction[0];
        float t_max_y = t_min + (voxel_max[1] - start[1]) / direction[1];
        float t_max_z = t_min + (voxel_max[2] - start[2]) / direction[2];

        float t_delta_x = this->leaf_size_[0] / static_cast<float> (fabs (direction[0]));
        float t_delta_y = this->leaf_size_[1] / static_cast<float> (fabs (direction[1]));
        float t_delta_z = this->leaf_size_[2] / static_cast<float> (fabs (direction[2]));

        // the index of the cloud (-1 if empty)
        int index = -1;

        //cout << endl << endl << "target_voxel : " << target_voxel[0] << " " << target_voxel[1] << " " << target_voxel[2] << endl;

        while ( (ijk[0] < this->max_b_[0]+1) && (ijk[0] >= this->min_b_[0]) &&
                (ijk[1] < this->max_b_[1]+1) && (ijk[1] >= this->min_b_[1]) &&
                (ijk[2] < this->max_b_[2]+1) && (ijk[2] >= this->min_b_[2]) )
        {
            //cout << "ijk : " << ijk[0] << " " << ijk[1] << " " << ijk[2] << endl;
            // add voxel to ray
            out_ray.push_back (ijk);

            // check if we reached target voxel
            int distance = abs(ijk[0]-target_voxel[0]) + abs(ijk[1]-target_voxel[1]) + abs(ijk[2]-target_voxel[2]);
            if (distance<=theta)
                return 0;

            // check if voxel is occupied
            index = this->getCentroidIndexAt (ijk);
            if (index != -1) {
                return 1;
            }

            // estimate next voxel
            if(t_max_x <= t_max_y && t_max_x <= t_max_z)
            {
                t_max_x += t_delta_x;
                ijk[0] += step_x;
            }
            else if(t_max_y <= t_max_z && t_max_y <= t_max_x)
            {
                t_max_y += t_delta_y;
                ijk[1] += step_y;
            }
            else
            {
                t_max_z += t_delta_z;
                ijk[2] += step_z;
            }
        }
        return 0;
    }
};

#endif // RAYTRACING_H
