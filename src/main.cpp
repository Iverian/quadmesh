#define _USE_MATH_DEFINES

#include <gm/shell.hpp>
#include <qmsh/config.hpp>
#include <qmsh/inout.hpp>
#include <qmsh/mesh.hpp>
#include <stp/parse.hpp>

#include <cxxopts.hpp>
#include <fmt/core.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/rapidjson.h>

#include <cmms/logging.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

static constexpr auto logger_id = "root";
static constexpr auto default_conf = "quadmeshgen-config.json";
static constexpr auto output_ext = "out.geo";

enum ExitCode : int {
    SUCCESS,
    INPUT_FILE_NOT_FOUND,
    INVALID_CONFIG,
    UNABLE_TO_PARSE_ARGS,
    NO_INPUT_FILE_PROVIDED,
    UNCAUGHT_EXCEPTION
};

struct CmdParameters {
    std::vector<gm::Shell> data;
    qmsh::Config conf;
    std::string output_path;
};

CmdParameters parse_args(spdlog::logger* const logger, int& argc,
                         char**& argv);
qmsh::Config parse_config(spdlog::logger* const logger,
                          const std::string& path);

int main(int argc, char** argv)
{
    auto logger = cmms::setup_logger(logger_id);
#ifndef NDEBUG
    spdlog::set_level(spdlog::level::level_enum::debug);
#else  // NDEBUG
    spdlog::set_level(spdlog::level::level_enum::warn);
#endif // NDEBUG

    try {
        auto param = parse_args(logger.get(), argc, argv);
        logger->debug("successfully read input arguments");

        std::vector<qmsh::Mesh> result(param.data.size());
        std::transform(std::begin(param.data), std::end(param.data),
                       std::begin(result), [&conf = param.conf](auto& sh) {
                           return qmsh::build_mesh(sh, conf);
                       });

        logger->debug("writing to file `{}`", param.output_path);
        // FIXME
        std::ofstream os(param.output_path);
        result[0].to_gmsh(os);
        os.close();
        // qmsh::json_export(result, param.output_path);
    } catch (const std::exception& err) {
        logger->critical("uncaught exception: {}", err.what());
        std::exit(UNCAUGHT_EXCEPTION);
    }

    return SUCCESS;
}

CmdParameters parse_args(spdlog::logger* const logger, int& argc, char**& argv)
{
    cxxopts::Options opts("QuadMeshGen", "Quadriliteral mesh generator");
    opts.add_options()(
        "c,config-path", "Path to config file",
        cxxopts::value<std::string>()->default_value(default_conf))(
        "i,input-path", "Path to input STEP file",
        cxxopts::value<std::string>())("o,output-path", "Path to output file",
                                       cxxopts::value<std::string>())(
        "h,help", "Print this message and exit");

    try {
        CmdParameters result;
        auto args = opts.parse(argc, argv);

        if (args.count("help")) {
            fmt::print("{}", opts.help());
            std::exit(SUCCESS);
        }

        if (!args.count("input-path")) {
            logger->critical("Input path is not provided");
            fmt::print("\n{}", opts.help());
            std::exit(NO_INPUT_FILE_PROVIDED);
        }

        auto input_path = args["input-path"].as<std::string>();
        if (!std::filesystem::exists(std::filesystem::path(input_path))) {
            logger->critical("provided input path `{}` does not exist",
                             input_path);
            std::exit(INPUT_FILE_NOT_FOUND);
        }
        result.data = stp::parse(input_path);

        if (args.count("output-path")) {
            result.output_path = args["output-path"].as<std::string>();
        } else {
            auto dot = input_path.rfind('.');
            result.output_path = input_path.substr(0, dot + 1) + output_ext;
        }
        logger->debug("output path: `{}`", result.output_path);

        auto conf_path = args["config-path"].as<std::string>();
        result.conf = parse_config(logger, conf_path);

        return result;
    } catch (const cxxopts::OptionException& err) {
        logger->critical("unable to parse options: {}", err.what(),
                         opts.help());
        fmt::print("\n{}", opts.help());
        std::exit(UNABLE_TO_PARSE_ARGS);
    }
}

qmsh::Config parse_config(spdlog::logger* const logger,
                          const std::string& path)
{
    qmsh::Config result;
    auto p = std::filesystem::absolute(std::filesystem::path(path));

    if (std::filesystem::exists(p)) {
        std::ifstream is(path);
        rapidjson::IStreamWrapper isw(is);
        rapidjson::Document d;
        rapidjson::ParseResult ok = d.ParseStream(isw);

        if (!ok) {
            logger->critical("unable to parse config: {} ({})",
                             rapidjson::GetParseError_En(ok.Code()),
                             ok.Offset());

            std::exit(INVALID_CONFIG);
        }

        auto& angtol = d["angle_tolerances"];
        if (angtol.Size() != result.angtol().size()) {
            logger->critical(
                "expected {} values in `angle_tolerances` list, got: {}",
                result.angtol().size(), angtol.Size());

            std::exit(INVALID_CONFIG);
        }
        for (size_t i = 0; i < result.angtol().size(); ++i) {
            auto v = angtol[i].GetDouble();
            if (v < 0 || v > M_PI / 2) {
                logger->critical(
                    "angle tolerance value is out of range: expected "
                    "{} in [{}, {}]",
                    v, 0, M_PI / 2);

                std::exit(INVALID_CONFIG);
            }
            result.angtol()[i] = v;
        }

        auto& discr_coeffs = d["discretize_coeffs"];
        if (discr_coeffs.Size() != 2) {
            logger->critical(
                "invalid number of discretize coefficients: expected "
                "2, got {}",
                discr_coeffs.Size());

            std::exit(INVALID_CONFIG);
        }
        result.discr_coeffs().first = discr_coeffs[0].GetUint64();
        result.discr_coeffs().second = discr_coeffs[1].GetUint64();
    } else {
        rapidjson::Document d;
        result.serialize(d, d.GetAllocator());

        std::ofstream of(path, std::ios_base::trunc);
        rapidjson::OStreamWrapper osw(of);
        rapidjson::PrettyWriter writer(osw);
        d.Accept(writer);
    }

    return result;
}
