// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef TACHIMAWARI_JOINT__UTILS__UTILS_HPP
#define TACHIMAWARI_JOINT__UTILS__UTILS_HPP

#include <iostream>
#include <string>

namespace tachimawari::joint::utils
{
std::string get_host_name();
std::string get_env(std::string env);
bool is_root();

bool is_directory_exist(std::string path);
bool create_directory(std::string path);

bool is_file_exist(std::string path);
bool create_file(std::string path);

std::string split_string(std::string s, std::string del = " ");

}  // namespace tachimawari::joint::utils

#endif  // TACHIMAWARI__JOINT__UTILS__UTILS_HPP
