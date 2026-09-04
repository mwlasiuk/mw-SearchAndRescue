#include <cave-traversal-tool/FileIO.h>

#include <happly.h>
#include <laszip_api.h>
#include <limits>
#include <spdlog/spdlog.h>

bool load_text_file(std::vector<char>& output, const std::filesystem::path& path)
{
    if (std::ifstream file = std::ifstream(path, std::ios::binary))
    {
        file.seekg(0, std::ios::end);
        const size_t size = static_cast<size_t>(file.tellg());
        file.seekg(0, std::ios::beg);

        output.resize(size + 1UL);
        file.read(output.data(), size);
        output[size] = 0x00;

        return true;
    }

    return false;
}

bool load_trajectory_csv_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth)
{
    struct CSVTrajectoryPoseMat33
    {
        uint64_t ts_lidar = 0ULL;
        uint64_t ts_linux = 0ULL;

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        float r0 = 0.0f;
        float r1 = 0.0f;
        float r2 = 0.0f;

        float r3 = 0.0f;
        float r4 = 0.0f;
        float r5 = 0.0f;

        float r6 = 0.0f;
        float r7 = 0.0f;
        float r8 = 0.0f;
    };

    trajectory_pose_positions    = {};
    trajectory_pose_orientations = {};

    if (std::ifstream file = std::ifstream(path))
    {
        std::vector<CSVTrajectoryPoseMat33> csv_poses{};

        size_t      line_index = 0;
        std::string line{};
        std::string token{};

        while (std::getline(file, line))
        {
            if (line.empty())
                continue;

            if (line_index % Nth != 0)
            {
                line_index++;
                continue;
            }

            std::stringstream      string_stream(line);
            CSVTrajectoryPoseMat33 csv_pose{};

            std::getline(string_stream, token, ',');
            csv_pose.ts_lidar = std::stoull(token);

            std::getline(string_stream, token, ',');
            csv_pose.ts_linux = std::stoull(token);

            std::getline(string_stream, token, ',');
            csv_pose.x = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.y = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.z = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r0 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r1 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r2 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r3 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r4 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r5 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r6 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r7 = std::stof(token);

            std::getline(string_stream, token, ',');
            csv_pose.r8 = std::stof(token);

            csv_poses.push_back(csv_pose);
            line_index++;
        }

        trajectory_pose_positions    = {};
        trajectory_pose_orientations = {};

        for (const auto& csv_pose : csv_poses)
        {
            Point point{};
            point.position.x = csv_pose.x;
            point.position.y = csv_pose.y;
            point.position.z = csv_pose.z;

            TrajectoryPoseOrientationMat33 orientation{};
            orientation.orientation = glm::mat3(
                csv_pose.r0, csv_pose.r1, csv_pose.r2,
                csv_pose.r3, csv_pose.r4, csv_pose.r5,
                csv_pose.r6, csv_pose.r7, csv_pose.r8);

            trajectory_pose_positions.push_back(point);
            trajectory_pose_orientations.push_back(orientation);
        }

        return true;
    }

    return false;
}

bool load_trajectory_bin_mat33(const std::filesystem::path& path, std::vector<Point>& trajectory_pose_positions, std::vector<TrajectoryPoseOrientationMat33>& trajectory_pose_orientations, const size_t Nth)
{
    trajectory_pose_positions    = {};
    trajectory_pose_orientations = {};

    if (std::ifstream file = std::ifstream(path, std::ios::in | std::ios::binary))
    {
        uint32_t trajectory_length = 0;
        file.read(reinterpret_cast<char*>(&trajectory_length), sizeof(uint32_t));

        std::vector<Point> all_positions(trajectory_length);
        file.read(reinterpret_cast<char*>(all_positions.data()), trajectory_length * sizeof(Point));

        std::vector<TrajectoryPoseOrientationMat33> all_orientations(trajectory_length);
        file.read(reinterpret_cast<char*>(all_orientations.data()), trajectory_length * sizeof(TrajectoryPoseOrientationMat33));

        size_t index = 0;
        for (uint32_t i = 0; i < trajectory_length; ++i)
        {
            if (Nth > 1 && (index % Nth) != 0)
            {
                ++index;
                continue;
            }

            trajectory_pose_positions.push_back(all_positions[i]);
            trajectory_pose_orientations.push_back(all_orientations[i]);
            ++index;
        }

        return true;
    }

    return false;
}

bool load_stretcher_ply(const std::filesystem::path& path, std::vector<ColorPoint>& points, std::vector<uint32_t>& indices)
{
    points  = {};
    indices = {};

    happly::PLYData ply(path.string());

    if (!ply.hasElement("vertex") || ply.getVertexPositions().empty())
    {
        spdlog::error("PLY file {} has no vertex positions!", path.string());
        return false;
    }

    const auto& vertexNames = ply.getElement("vertex").getPropertyNames();
    if (!(std::count(vertexNames.begin(), vertexNames.end(), "red") &&
          std::count(vertexNames.begin(), vertexNames.end(), "green") &&
          std::count(vertexNames.begin(), vertexNames.end(), "blue")))
    {
        spdlog::error("PLY file {} has no vertex colors!", path.string());
        return false;
    }

    if (!ply.hasElement("face") || ply.getFaceIndices().empty())
    {
        spdlog::error("PLY file {} has no face indices!", path.string());
        return false;
    }

    const auto positions = ply.getVertexPositions();
    const auto colors    = ply.getVertexColors();

    points.clear();
    points.reserve(positions.size());

    for (size_t i = 0; i < positions.size(); i++)
    {
        ColorPoint pt;
        pt.position.x = static_cast<float>(positions[i][0]);
        pt.position.y = static_cast<float>(positions[i][1]);
        pt.position.z = static_cast<float>(positions[i][2]);

        pt.color.x = colors[i][0];
        pt.color.y = colors[i][1];
        pt.color.z = colors[i][2];

        points.push_back(pt);
    }

    const auto faceIndices = ply.getFaceIndices();

    indices.clear();

    for (const auto& f : faceIndices)
    {
        if (f.size() != 3)
        {
            spdlog::error("PLY file {} contains a non-triangle face!", path.string());
            return false;
        }

        indices.push_back(f[0]);
        indices.push_back(f[1]);
        indices.push_back(f[2]);
    }

    return true;
}

bool load_cave_laz(const std::filesystem::path& path, std::vector<PointIntensity>& points)
{
    points.clear();

    laszip_POINTER laszip_reader = nullptr;
    if (laszip_create(&laszip_reader))
    {
        spdlog::error("Failed to create LASzip reader");
        return false;
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, path.string().c_str(), &is_compressed))
    {
        laszip_CHAR* error_msg = nullptr;
        laszip_get_error(laszip_reader, &error_msg);
        spdlog::error("Failed to open LAZ/LAS file: {} - {}", path.string(), error_msg ? error_msg : "Unknown error");
        laszip_destroy(laszip_reader);
        return false;
    }

    laszip_header_struct* header = nullptr;
    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        spdlog::error("Failed to get LASzip header pointer");
        laszip_close_reader(laszip_reader);
        laszip_destroy(laszip_reader);
        return false;
    }

    laszip_I64 num_points = (header->number_of_point_records ? header->number_of_point_records : header->extended_number_of_point_records);

    laszip_point_struct* point = nullptr;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        spdlog::error("Failed to get LASzip point pointer");
        laszip_close_reader(laszip_reader);
        laszip_destroy(laszip_reader);
        return false;
    }

    spdlog::info("Loading LAZ file '{}': {} points (compressed: {})", path.string(), num_points, is_compressed ? "yes" : "no");

    points.reserve(static_cast<size_t>(num_points));

    const double x_scale  = header->x_scale_factor;
    const double y_scale  = header->y_scale_factor;
    const double z_scale  = header->z_scale_factor;
    const double x_offset = header->x_offset;
    const double y_offset = header->y_offset;
    const double z_offset = header->z_offset;

    for (laszip_I64 i = 0; i < num_points; ++i)
    {
        if (laszip_read_point(laszip_reader))
        {
            spdlog::error("Failed to read point at index {}", i);
            break;
        }

        const uint8_t intensity = static_cast<uint8_t>(point->intensity);

        PointIntensity pt{};
        pt.position.x = static_cast<float>(point->X * x_scale + x_offset);
        pt.position.y = static_cast<float>(point->Y * y_scale + y_offset);
        pt.position.z = static_cast<float>(point->Z * z_scale + z_offset);
        pt.intensity  = float(intensity) / 255.0f;

        points.push_back(pt);
    }

    laszip_close_reader(laszip_reader);
    laszip_destroy(laszip_reader);

    spdlog::info("Loaded {} points from LAZ file", points.size());

    return !points.empty();
}
