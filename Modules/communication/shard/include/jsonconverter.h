#ifndef JSONCONVERTER_H
#define JSONCONVERTER_H

#include <Struct.hpp>
#include <nlohmann/json.hpp>

// 辅助函数，从 JSON 对象中获取数据，如果键不存在则返回默认值  值或者容器使用该函数
template <typename T>
static T getValueFromJSON(const nlohmann::json& json, const std::string& key, T defaultValue);

// 辅助函数，从 JSON 对象中获取数据，如果键不存在则返回默认值  定长数组使用该函数
template <typename T, std::size_t N>
std::array<T, N> getValueFromJSON(const nlohmann::json& json, const std::string& key, const std::array<T, N>& defaultValue);

class JsonConverter
{
public:
    /*------------------------------将JSON转换为结构体----------------------------*/
    // MSG_ID 1 JSON转换为UAVState
    static void fromJson(const nlohmann::json& json, struct UAVState& uav_state);
    // MSG_ID 3 JSON转换为TextInfo
    static void fromJson(const nlohmann::json& json, struct TextInfo& text_info);
    // MSG_ID 4 JSON转换为GimbalState
    static void fromJson(const nlohmann::json& json, struct GimbalState& gimbal_state);
    // MSG_ID 5 JSON转换为VisionDiff
    static void fromJson(const nlohmann::json& json, struct VisionDiff& vision_diff);
    // MSG_ID 6 JSON转换为Heartbeat
    static void fromJson(const nlohmann::json& json, struct Heartbeat& heartbeat);
    // MSG_ID 7 JSON转换为UGVState
    static void fromJson(const nlohmann::json& json, struct UGVState& ugv_state);
    // MSG_ID 8 JSON转换为MultiDetectionInfo
    static void fromJson(const nlohmann::json& json, struct MultiDetectionInfo& multi_detection_info);
    // MSG_ID 9 JSON转换为UAVControlState
    static void fromJson(const nlohmann::json& json, struct UAVControlState& uav_contorl_state);
    // MSG_ID 101 JSON转换为SwarmCommand
    static void fromJson(const nlohmann::json& json, struct SwarmCommand& swarm_command);
    // MSG_ID 102 JSON转换为GimbalControl
    static void fromJson(const nlohmann::json& json, struct GimbalControl& gimbal_control);
    // MSG_ID 103 JSON转换为GimbalService
    static void fromJson(const nlohmann::json& json, struct GimbalService& gimbal_service);
    // MSG_ID 104 JSON转换为WindowPosition
    static void fromJson(const nlohmann::json& json, struct WindowPosition& window_position);
    // MSG_ID 105 JSON转换为UGVCommand
    static void fromJson(const nlohmann::json& json, struct UGVCommand& ugv_command);
    // MSG_ID 106 JSON转换为GimbalParamSet
    static void fromJson(const nlohmann::json& json, struct GimbalParamSet& gimbal_param_set);
    // MSG_ID 107 JSON转换为ImageData
    static void fromJson(const nlohmann::json& json, struct ImageData& image_data);
    // MSG_ID 108 JSON转换为UAVCommand
    static void fromJson(const nlohmann::json& json, struct UAVCommand& uav_command);
    // MSG_ID 109 JSON转换为UAVSetup
    static void fromJson(const nlohmann::json& json, struct UAVSetup& uav_setup);
    // MSG_ID 110 JSON转换为ParamSettings
    static void fromJson(const nlohmann::json& json, struct ParamSettings& param_settings);
    // MSG_ID 111 JSON转换为Bspline
    static void fromJson(const nlohmann::json& json, struct Bspline& bspline);
    // MSG_ID 112 JSON转换为MultiBsplines
    static void fromJson(const nlohmann::json& json, struct MultiBsplines& multi_bsplines);
    // MSG_ID 113 JSON转换为CustomDataSegment_1
    static void fromJson(const nlohmann::json& json, struct CustomDataSegment_1& custom_data_segment_1);
    // MSG_ID 201 JSON转换为ConnectState
    static void fromJson(const nlohmann::json& json, struct ConnectState& connect_state);
    // MSG_ID 202 JSON转换为ModeSelection
    static void fromJson(const nlohmann::json& json, struct ModeSelection& mode_selection);
    // MSG_ID 255 JSON转换为Goal
    static void fromJson(const nlohmann::json& json, struct Goal& goal);

    /*------------------------------将结构体转换为JSON----------------------------*/
    // MSG_ID 1 UAVState转换为JSON
    static void toJson(const struct UAVState& uav_state, nlohmann::json& json);
    // MSG_ID 3 TextInfo转换为JSON
    static void toJson(const struct TextInfo& text_info, nlohmann::json& json);
    // MSG_ID 4 GimbalState转换为JSON
    static void toJson(const struct GimbalState& gimbal_state, nlohmann::json& json);
    // MSG_ID 5 VisionDiff转换为JSON
    static void toJson(const struct VisionDiff& vision_diff, nlohmann::json& json);
    // MSG_ID 6 Heartbeat转换为JSON
    static void toJson(const struct Heartbeat& heartbeat, nlohmann::json& json);
    // MSG_ID 7 UGVState转换为JSON
    static void toJson(const struct UGVState& ugv_state, nlohmann::json& json);
    // MSG_ID 8 MultiDetectionInfo转换为JSON
    static void toJson(const struct MultiDetectionInfo& multi_detection_info, nlohmann::json& json);
    // MSG_ID 9 UAVControlState转换为JSON
    static void toJson(struct UAVControlState& uav_contorl_state,nlohmann::json& json);
    // MSG_ID 101 SwarmCommand转换为JSON
    static void toJson(struct SwarmCommand& swarm_command,nlohmann::json& json);
    // MSG_ID 102 GimbalControl转换为JSON
    static void toJson(const struct GimbalControl& gimbal_control, nlohmann::json& json);
    // MSG_ID 103 GimbalService转换为JSON
    static void toJson(const struct GimbalService& gimbal_service, nlohmann::json& json);
    // MSG_ID 104 WindowPosition转换为JSON
    static void toJson(const struct WindowPosition& window_position, nlohmann::json& json);
    // MSG_ID 105 UGVCommand转换为JSON
    static void toJson(const struct UGVCommand& ugv_command, nlohmann::json& json);
    // MSG_ID 106 GimbalParamSet转换为JSON
    static void toJson(const struct GimbalParamSet& gimbal_param_set, nlohmann::json& json);
    // MSG_ID 107 ImageData转换为JSON
    static void toJson(const struct ImageData& image_data, nlohmann::json& json);
    // MSG_ID 108 UAVCommand转换为JSON
    static void toJson(const struct UAVCommand& uav_command, nlohmann::json& json);
    // MSG_ID 109 UAVSetup转换为JSON
    static void toJson(const struct UAVSetup& uav_setup, nlohmann::json& json);
    // MSG_ID 110 ParamSettings转换为JSON
    static void toJson(const struct ParamSettings& param_settings, nlohmann::json& json);
    // MSG_ID 111 Bspline转换为JSON
    static void toJson(const struct Bspline& bspline, nlohmann::json& json);
    // MSG_ID 112 MultiBsplines转换为JSON
    static void toJson(const struct MultiBsplines& multi_bsplines, nlohmann::json& json);
    // MSG_ID 113 CustomDataSegment_1转换为JSON
    static void toJson(const struct CustomDataSegment_1& custom_data_segment_1, nlohmann::json& json);
    // MSG_ID 201 ConnectState转换为JSON
    static void toJson(const struct ConnectState& connect_state, nlohmann::json& json);
    // MSG_ID 202 ModeSelection转换为JSON
    static void toJson(const struct ModeSelection& mode_selection, nlohmann::json& json);
    // MSG_ID 255 Goal转换为JSON
    static void toJson(const struct Goal& goal, nlohmann::json& json);
};

template <typename T>
T getValueFromJSON(const nlohmann::json& json, const std::string& key, T defaultValue)
{
    if (json.contains(key) && (json[key].is_number_integer() || json[key].is_array())) {
        return json[key].get<T>();
    }
    return defaultValue;
}

template <typename T, std::size_t N>
std::array<T, N> getValueFromJSON(const nlohmann::json& json, const std::string& key, const std::array<T, N>& defaultValue) 
{
    if (json.contains(key) && json[key].is_array()) {
        const auto& jsonArray = json[key];
        if (jsonArray.size() == N) {
            std::array<T, N> values;
            for (std::size_t i = 0; i < N; ++i) {
                const auto& element = jsonArray[i];
                if (element.is_number()) {
                    values[i] = element.get<T>();
                } else {
                    return defaultValue;
                }
            }
            return values;
        }
    }
    return defaultValue;
}
#endif // JSONCONVERTER_H
