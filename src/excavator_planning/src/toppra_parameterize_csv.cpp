// Convert a joint-space path CSV into a time-parameterized joint trajectory
// CSV using the project's TOPP-RA time parameterizer.
//
// Input rows may contain only joint positions, an optional leading path
// index/time column followed by positions, or a full trajectory row. Only the
// positions are used; TOPP-RA computes a rest-to-rest time law from them.
//
// Output is written to stdout (or --output) using the same format as
// sample_trajectory.csv:
//
//   joint_names,<joint_1>,...,<joint_N>
//   t,positions...,velocities...,accelerations...

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "excavator_planning/toppra_time_parameterizer.hpp"

namespace {

using tramcar::planning::JointPath;
using tramcar::planning::JointPositions;
using tramcar::planning::TimeParameterizationRequest;
using tramcar::planning::TimeParameterizationResponse;
using tramcar::planning::ToppraTimeParameterizer;

std::string Trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

std::vector<std::string> SplitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(Trim(field));
  }
  return fields;
}

bool ParseDouble(const std::string& text, double* value) {
  if (value == nullptr) {
    return false;
  }
  std::istringstream stream(text);
  double parsed = 0.0;
  stream >> parsed;
  if (!stream || !stream.eof()) {
    return false;
  }
  *value = parsed;
  return true;
}

std::vector<double> ParseList(const std::string& text,
                              const std::string& option) {
  std::vector<double> values;
  std::istringstream stream(text);
  std::string field;
  while (std::getline(stream, field, ',')) {
    double value = 0.0;
    if (!ParseDouble(Trim(field), &value) || !std::isfinite(value) ||
        value <= 0.0) {
      throw std::runtime_error(option +
                               " must be a comma-separated list of positive "
                               "finite numbers");
    }
    values.push_back(value);
  }
  if (values.empty()) {
    throw std::runtime_error(option + " must not be empty");
  }
  return values;
}

void PrintUsage() {
  std::cout
      << "Usage: toppra_parameterize_csv --input PATH [options]\n"
      << "\n"
      << "Reads joint path points and writes a TOPP-RA time-parameterized\n"
      << "joint trajectory CSV to stdout (or --output).\n"
      << "\n"
      << "Options:\n"
      << "  --input PATH                 Path CSV (required)\n"
      << "  --output PATH                Output CSV (default: stdout)\n"
      << "  --grid-points N              TOPP-RA intervals, samples = N+1\n"
      << "                               (default: 200)\n"
      << "  --max-velocity v1,v2,...     Joint velocity limits (rad/s)\n"
      << "  --max-acceleration a1,a2,... Joint acceleration limits (rad/s^2)\n"
      << "  -h, --help                   Show this help\n"
      << "\n"
      << "Defaults for --max-velocity / --max-acceleration are provided for\n"
      << "the four standard excavator joints\n"
      << "(swing, boom, stick, bucket).\n";
}

std::string RequireValue(int* index, int argc, char** argv,
                         const std::string& option) {
  if (*index + 1 >= argc) {
    throw std::runtime_error("missing value for " + option);
  }
  ++(*index);
  return argv[*index];
}

// The shared time parameterizer contains diagnostic std::cout output used by
// the planning server. Redirect it while this tool runs so stdout contains
// nothing but the output CSV.
class ScopedStreamRedirect {
 public:
  ScopedStreamRedirect(std::ostream& stream, std::streambuf* buffer)
      : stream_(stream), saved_(stream.rdbuf(buffer)) {}
  ~ScopedStreamRedirect() { stream_.rdbuf(saved_); }

  ScopedStreamRedirect(const ScopedStreamRedirect&) = delete;
  ScopedStreamRedirect& operator=(const ScopedStreamRedirect&) = delete;

 private:
  std::ostream& stream_;
  std::streambuf* saved_;
};

}  // namespace

int main(int argc, char** argv) {
  std::string input_path;
  std::string output_path;
  int grid_points = 200;
  std::string velocity_text;
  std::string acceleration_text;

  try {
    for (int index = 1; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--input" || argument == "-i") {
        input_path = RequireValue(&index, argc, argv, argument);
      } else if (argument == "--output" || argument == "-o") {
        output_path = RequireValue(&index, argc, argv, argument);
      } else if (argument == "--grid-points") {
        grid_points = std::stoi(RequireValue(&index, argc, argv, argument));
      } else if (argument == "--max-velocity") {
        velocity_text = RequireValue(&index, argc, argv, argument);
      } else if (argument == "--max-acceleration") {
        acceleration_text = RequireValue(&index, argc, argv, argument);
      } else if (argument == "-h" || argument == "--help") {
        PrintUsage();
        return 0;
      } else {
        std::cerr << "unknown argument: " << argument << "\n";
        PrintUsage();
        return 2;
      }
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 2;
  }

  if (input_path.empty()) {
    std::cerr << "--input is required\n";
    PrintUsage();
    return 2;
  }
  if (grid_points < 2) {
    std::cerr << "--grid-points must be at least 2\n";
    return 2;
  }

  try {
    std::ifstream stream(input_path);
    if (!stream) {
      std::cerr << "cannot open input file: " << input_path << "\n";
      return 2;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
      line = Trim(line);
      if (!line.empty()) {
        lines.push_back(line);
      }
    }
    if (lines.empty()) {
      std::cerr << "input file is empty: " << input_path << "\n";
      return 2;
    }

    std::vector<std::string> header = SplitCsv(lines.front());
    // Tolerate trailing empty columns in the header.
    while (header.size() > 1 && header.back().empty()) {
      header.pop_back();
    }
    if (header.size() < 2 || header.front() != "joint_names") {
      std::cerr << "CSV must start with a 'joint_names,...' header row\n";
      return 2;
    }
    JointPath path;
    for (std::size_t index = 1; index < header.size(); ++index) {
      if (header[index].empty()) {
        std::cerr << "CSV contains an empty joint name\n";
        return 2;
      }
      path.joint_names.push_back(header[index]);
    }
    const std::size_t joint_count = path.joint_names.size();

    for (std::size_t line_index = 1; line_index < lines.size();
         ++line_index) {
      std::vector<std::string> fields = SplitCsv(lines[line_index]);
      while (!fields.empty() && fields.back().empty()) {
        fields.pop_back();
      }
      std::size_t offset = 0;
      if (fields.size() == joint_count) {
        offset = 0;  // positions only
      } else if (fields.size() == joint_count + 1) {
        offset = 1;  // optional path index/time followed by positions
      } else if (fields.size() == 3 * joint_count + 1) {
        offset = 1;  // full trajectory row; use positions only
      } else {
        std::cerr << "line " << line_index + 1 << ": expected "
                  << joint_count << ", " << joint_count + 1 << " or "
                  << 3 * joint_count + 1 << " columns, got "
                  << fields.size() << "\n";
        return 2;
      }
      JointPositions positions(joint_count);
      for (std::size_t joint = 0; joint < joint_count; ++joint) {
        double value = 0.0;
        if (!ParseDouble(fields[offset + joint], &value) ||
            !std::isfinite(value)) {
          std::cerr << "line " << line_index + 1 << ", joint " << joint + 1
                    << ": invalid joint position\n";
          return 2;
        }
        positions[joint] = value;
      }
      path.points.push_back(std::move(positions));
    }
    if (path.points.size() < 2) {
      std::cerr << "input path must contain at least two joint points\n";
      return 2;
    }

    TimeParameterizationRequest request;
    request.path = std::move(path);
    if (velocity_text.empty() && acceleration_text.empty()) {
      if (request.path.points.front().size() != 4) {
        std::cerr << "input has " << request.path.points.front().size()
                  << " joints; --max-velocity and --max-acceleration are "
                     "required\n";
        return 2;
      }
      request.max_velocity.assign({0.5, 0.3, 0.4, 0.5});
      request.max_acceleration.assign({1.0, 0.6, 0.8, 1.0});
    } else {
      request.max_velocity = ParseList(velocity_text, "--max-velocity");
      request.max_acceleration =
          ParseList(acceleration_text, "--max-acceleration");
      if (request.max_velocity.size() != request.path.joint_names.size() ||
          request.max_acceleration.size() !=
              request.path.joint_names.size()) {
        std::cerr << "limit list length must match the number of joints ("
                  << request.path.joint_names.size() << ")\n";
        return 2;
      }
    }

    ToppraTimeParameterizer parameterizer(
        static_cast<std::size_t>(grid_points));
    TimeParameterizationResponse response;
    {
      std::ostringstream discarded_diagnostics;
      ScopedStreamRedirect redirect(std::cout,
                                    discarded_diagnostics.rdbuf());
      response = parameterizer.Parameterize(request);
    }
    if (!response.success) {
      std::cerr << "TOPP-RA parameterization failed: " << response.message
                << "\n";
      return 1;
    }

    std::ofstream output_file;
    std::ostream* output = &std::cout;
    if (!output_path.empty()) {
      output_file.open(output_path);
      if (!output_file) {
        std::cerr << "cannot open output file: " << output_path << "\n";
        return 2;
      }
      output = &output_file;
    }

    *output << "joint_names";
    for (const std::string& name : response.trajectory.joint_names) {
      *output << ',' << name;
    }
    *output << '\n';
    *output << std::fixed << std::setprecision(9);
    for (const auto& point : response.trajectory.points) {
      *output << point.t;
      for (double value : point.positions) {
        *output << ',' << value;
      }
      for (double value : point.velocities) {
        *output << ',' << value;
      }
      for (double value : point.accelerations) {
        *output << ',' << value;
      }
      *output << '\n';
    }
    std::cerr << response.message << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << "\n";
    return 2;
  }
}
