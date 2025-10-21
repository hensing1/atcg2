#pragma once

#include <Renderer/Shader.h>
#include <unordered_map>
#include <string>
#include <memory>

#include <filesystem>

/**
 * @brief This class manages all shaders used for the application
 * When using custom shaders, you have to add them via 'addShader' to get hot reloading or
 * use addShaderFromPath to also handle direct shader loading
 */
namespace atcg
{
/**
 * @brief This class handles all shaders
 */
class ShaderManager
{
public:
    /**
     * @brief Add a shader
     *
     * @param name Name of the shader
     * @param shader The shader
     */
    inline static void addShader(const std::string& name, const std::shared_ptr<Shader>& shader)
    {
        s_instance->addShaderImpl(name, shader);
    }

    /**
     * @brief Add a shader by loading it from file
     *
     * @param name The name of the .vs, .fs and optionally .gs file (without file ending)
     */
    inline static void addShaderFromName(const std::string& name) { s_instance->addShaderFromNameImpl(name); }

    /**
     * @brief Get the Shader object
     *
     * @param name The name
     * @return const std::shared_ptr<Shader>& The shader
     */
    inline static const std::shared_ptr<Shader>& getShader(const std::string& name)
    {
        return s_instance->getShaderImpl(name);
    }

    /**
     * @brief This gets called by the application. Don't call manually
     */
    inline static void onUpdate() { s_instance->onUpdateImpl(); }

private:
    void addShaderImpl(const std::string& name, const std::shared_ptr<Shader>& shader);
    void addShaderFromNameImpl(const std::string& name);
    const std::shared_ptr<Shader>& getShaderImpl(const std::string& name);
    void onUpdateImpl();
    static ShaderManager* s_instance;

    std::unordered_map<std::string, std::shared_ptr<Shader>> _shader;
    std::unordered_map<std::string, std::filesystem::file_time_type> _time_stamps;
};
}    // namespace atcg