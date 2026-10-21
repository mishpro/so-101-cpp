// Проверки валидации send_config.json: правка конфигурации должна отвергаться
// с внятным сообщением, а не молча.

#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include "control_config.hpp"
#include "test_support.hpp"

namespace {

using json = nlohmann::json;

json valid_config_object() {
    return json{{"port", "/dev/ttyACM0"},
                {"speed", 600},
                {"joint_speed_divisor", 1.2},
                {"urdf_path", "~/develop/urdf/so101_irs_ksant.urdf"},
                {"cartesian_step_m", 0.01},
                {"end_effector", "gripper_frame_link"},
                {"ik_damping", 0.0001},
                {"ik_max_iterations", 100},
                {"ik_position_tolerance_m", 0.005},
                {"ik_max_joint_step_rad", 0.1}};
}

// Рабочий каталог теста — каталог сборки, поэтому фиксированного имени
// достаточно и не нужен deprecated std::tmpnam.
std::string write_temporary(const json& body) {
    const std::string path = "test_config_input.json";
    std::ofstream file(path);
    file << body.dump(2);
    file.close();
    return path;
}

std::string write_raw(const std::string& body) {
    const std::string path = "test_config_input.json";
    std::ofstream file(path);
    file << body;
    file.close();
    return path;
}

void check_rejected(const json& body, const char* what) {
    const std::string path = write_temporary(body);
    ControlConfig config{};
    CHECK_MSG(!load_config(path, config), what);
    std::remove(path.c_str());
}

void check_rejected_raw(const std::string& body, const char* what) {
    const std::string path = write_raw(body);
    ControlConfig config{};
    CHECK_MSG(!load_config(path, config), what);
    std::remove(path.c_str());
}

json with_field(const char* key, const json& value) {
    json body = valid_config_object();
    body[key] = value;
    return body;
}

}  // namespace

// Скорость сустава ограничивается диапазоном мотора.
//
// Делитель берётся из конфига и может быть меньше единицы: тогда для дальних
// суставов скорость уходит за пределы регистра, а приведение double к
// uint16_t вне диапазона — неопределённое поведение.
void test_joint_speed_is_clamped() {
    // Обычный случай: суставы идут медленнее первого.
    CHECK(joint_speed(600, 1.2, 0) == 600);
    CHECK(joint_speed(600, 1.2, 1) == 500);
    // 600 / 1.2^2 = 416.67, округление даёт 417.
    CHECK(joint_speed(600, 1.2, 2) == 417);

    // Делитель меньше единицы: скорость растёт, но не выходит за пределы
    // регистра и не оборачивается в ноль.
    const uint16_t fast = joint_speed(600, 0.1, 5);
    CHECK(fast >= kMinMotorSpeed);
    CHECK(fast <= kMaxMotorSpeed);

    // Совсем большой делитель: скорость не должна упасть ниже минимума.
    CHECK(joint_speed(600, 100.0, 3) == kMinMotorSpeed);
}

int main() {
    test_joint_speed_is_clamped();

    // Корректный конфиг должен приниматься и разворачивать ~.
    {
        const std::string path = write_temporary(valid_config_object());
        ControlConfig config{};
        CHECK_MSG(load_config(path, config), "корректный конфиг");
        CHECK(config.urdf_path.find('~') == std::string::npos);
        CHECK(config.speed == 600);
        CHECK(config.ik_max_iterations == 100);
        std::remove(path.c_str());
    }

    check_rejected(with_field("ik_damping", -0.0001), "отрицательный damping");
    check_rejected(with_field("ik_damping", 0), "нулевой damping");
    check_rejected(with_field("ik_max_iterations", 0), "нулевые итерации");
    check_rejected(with_field("ik_max_iterations", -5), "отрицательные итерации");
    check_rejected(with_field("ik_max_iterations", 100000),
                   "слишком много итераций");
    check_rejected(with_field("cartesian_step_m", 0), "нулевой шаг");
    check_rejected(with_field("cartesian_step_m", -0.01),
                   "отрицательный шаг");
    check_rejected(with_field("ik_position_tolerance_m", 0),
                   "нулевой допуск позиции");
    check_rejected(with_field("ik_max_joint_step_rad", 0),
                   "нулевой лимит шага сустава");
    check_rejected(with_field("joint_speed_divisor", 0.0),
                   "нулевой делитель скорости");
    check_rejected(with_field("speed", 0), "нулевая скорость");
    check_rejected(with_field("speed", 5000), "скорость вне диапазона");
    check_rejected(with_field("end_effector", ""), "пустой end effector");
    check_rejected(with_field("port", ""), "пустой порт");

    // Отсутствующие ключи.
    {
        json body = valid_config_object();
        body.erase("ik_damping");
        check_rejected(body, "отсутствует ik_damping");
    }
    {
        json body = valid_config_object();
        body.erase("ik_max_joint_step_rad");
        check_rejected(body, "отсутствует ik_max_joint_step_rad");
    }

    // Некорректный ввод не должен ронять программу.
    check_rejected_raw("{ это не json", "битый json");
    check_rejected_raw("", "пустой файл");
    check_rejected_raw("[]", "не объект");

    // Комментарии в конфиге не поддерживаются — README это отражает.
    check_rejected_raw(
        "{\n  // комментарий\n  \"port\": \"/dev/ttyACM0\"\n}\n",
        "комментарий в json");

    // Несуществующий файл.
    {
        ControlConfig config{};
        CHECK(!load_config("no_such_config_file.json", config));
    }

    CHECK(expand_home("/abs/path") == "/abs/path");

    return so101_test::finish("control_config");
}