// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EASYNAV_COMMON__SINGLETON_H_
#define EASYNAV_COMMON__SINGLETON_H_

#include <memory>
#include <mutex>
#include <utility>

namespace easynav
{

template<class C>
class Singleton
{
public:
  template<typename ... Args>
  static C * getInstance(Args &&... args)
  {
    std::call_once(init_flag_, [&]() {
        instance_ = std::make_unique<C>(std::forward<Args>(args)...);
    });
    return instance_.get();
  }

  static void removeInstance()
  {
    instance_.reset();
    init_flag_ = std::once_flag();
  }

  template<typename ... Args>
  static C & get(Args &&... args)
  {
    getInstance(std::forward<Args>(args)...);
    return *instance_;
  }

protected:
  Singleton() = default;
  ~Singleton() = default;

  Singleton(const Singleton &) = delete;
  Singleton & operator=(const Singleton &) = delete;

private:
  static std::unique_ptr<C> instance_;
  static std::once_flag init_flag_;
};

template<class C>
std::unique_ptr<C> Singleton<C>::instance_ = nullptr;

template<class C>
std::once_flag Singleton<C>::init_flag_;

#define SINGLETON_DEFINITIONS(ClassName) \
public: \
  static ClassName * getInstance() \
  { \
    return ::easynav::Singleton<ClassName>::getInstance(); \
  } \
  template<typename ... Args> \
  static ClassName * getInstance(Args && ... args) \
  { \
    return ::easynav::Singleton<ClassName>::getInstance( \
      std::forward<Args>(args)...); \
  }

}  // namespace easynav

#endif  // EASYNAV_COMMON__SINGLETON_H_
