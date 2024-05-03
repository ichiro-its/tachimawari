// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "tachimawari/joint/utils/utils.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace tachimawari::joint::utils
{

std::string get_host_name()
{
  char hostname[32];
  if (gethostname(hostname, 32) != 0) {
    return "";
  }

  return hostname;
}

std::string get_env(std::string env) { return getenv(env.c_str()); }

bool is_root() { return getuid() == 0; }

bool is_directory_exist(std::string path)
{
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return false;
  }

  return (st.st_mode & S_IFMT) == S_IFDIR;
}

bool create_directory(std::string path)
{
  std::replace(path.begin(), path.end(), '/', ' ');

  std::vector<std::string> directories;
  std::stringstream ss(path);

  std::string str;
  while (ss >> str) {
    directories.push_back(str);
  }

  ss.str("");
  ss.clear();
  for (std::vector<std::string>::iterator it = directories.begin(); it != directories.end(); it++) {
    ss << *it << "/";

    if (!is_directory_exist(ss.str())) {
      if (mkdir(ss.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0) {
        return false;
      }
    }
  }

  return true;
}

bool is_file_exist(std::string path)
{
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return false;
  }

  return (st.st_mode & S_IFMT) == S_IFREG;
}

bool create_file(std::string path)
{
  std::size_t slash = path.find_last_of("/");
  if (slash < path.size()) {
    std::string directory = path.substr(0, (slash == path.size()) ? 0 : slash);

    if (!is_directory_exist(directory)) {
      if (!create_directory(directory)) {
        return false;
      }
    }
  }

  std::ofstream output;
  output.open(path);
  if (output.is_open() == false) {
    return false;
  }

  output.close();

  return true;
}

std::string split_string(std::string s, std::string del)
{
  int start = 0;
  int end = s.find(del);

  while (end != -1) {
    // cout << s.substr(start, end - start) << endl;
    start = end + del.size();
    end = s.find(del, start);
  }

  return s.substr(start, end - start);
}

}  // namespace tachimawari::joint::utils
