#include "drawing.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace so101 {
namespace {

constexpr double kPiDrawing = 3.14159265358979323846;

}  // namespace

std::vector<Eigen::Vector3d> circle_targets(const Plane& plane,
                                             const CircleSpec& circle,
                                             double press_m,
                                             const Eigen::Vector3d& current_position) {
	std::vector<Eigen::Vector3d> targets;
	if (circle.points == 0 || circle.radius_m <= 0.0) return targets;
	targets.reserve(circle.points);

	// Ближайшая к текущему положению точка окружности: угол между
	// радиусом к текущей позиции и осью X.
	const double current_angle = std::atan2(current_position.y() - circle.center_y_m,
	                                        current_position.x() - circle.center_x_m);
	const double step = 2.0 * kPiDrawing / static_cast<double>(circle.points);
	const double start_angle =
	    std::round(current_angle / step) * step;

	// Высота цели: плоскость минус нажатие. Карандаш должен давить на
	// бумагу, иначе он просто не оставит следа.
	const double height = plane.tcp_height_m - press_m;

	for (std::size_t index = 0; index < circle.points; ++index) {
		const double angle = start_angle + step * static_cast<double>(index);
		Eigen::Vector3d target;
		target.x() = circle.center_x_m + circle.radius_m * std::cos(angle);
		target.y() = circle.center_y_m + circle.radius_m * std::sin(angle);
		target.z() = height;
		targets.push_back(target);
	}
	return targets;
}

bool save_plane(const std::string& path, const Plane& plane, std::string& error) {
	nlohmann::json document;
	document["tcp_height_m"] = plane.tcp_height_m;
	document["contact_x_m"] = plane.contact_x_m;
	document["contact_y_m"] = plane.contact_y_m;
	document["tilt_deg"] = plane.tilt_deg;

	// Атомарная запись: временный файл рядом с целью, потом переименование.
	// Читатель либо видит старую плоскость, либо новую, но не половину.
	const std::string temporary = path + ".tmp";
	{
		std::ofstream file(temporary, std::ios::trunc);
		if (!file) {
			error = "Cannot open " + temporary + " for writing";
			return false;
		}
		file << document.dump(2) << '\n';
		if (!file) {
			error = "Failed to write " + temporary;
			return false;
		}
	}

	std::error_code code;
	std::filesystem::rename(temporary, path, code);
	if (code) {
		error = "Cannot rename " + temporary + " to " + path + ": " + code.message();
		std::remove(temporary.c_str());
		return false;
	}
	return true;
}

bool load_plane(const std::string& path, Plane& plane, std::string& error) {
	std::ifstream file(path);
	if (!file) {
		error = "Cannot open " + path;
		return false;
	}

	nlohmann::json document;
	try {
		file >> document;
	} catch (const std::exception& exception) {
		error = "Failed to parse " + path + ": " + exception.what();
		return false;
	}

	try {
		plane.tcp_height_m = document.at("tcp_height_m").get<double>();
		plane.contact_x_m = document.at("contact_x_m").get<double>();
		plane.contact_y_m = document.at("contact_y_m").get<double>();
		plane.tilt_deg = document.at("tilt_deg").get<double>();
	} catch (const std::exception& exception) {
		error = "Incomplete plane file " + path + ": " + exception.what();
		return false;
	}
	return true;
}

}  // namespace so101
