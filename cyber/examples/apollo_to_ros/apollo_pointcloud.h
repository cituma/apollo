/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef APOLLO_POINT_CLOUD_H_
#define APOLLO_POINT_CLOUD_H_

#include <iostream>
#include <vector>

struct ApolloPoint {
    float x;
    float y;
    float z;
};

class ApolloPointCloud {
public:
    ApolloPointCloud() : width(0), height(0) {
    }
    ApolloPointCloud(uint32_t w, uint32_t h) : width(w), height(h) {
        points.resize(w*h);
    }
    ~ApolloPointCloud() {
        points.clear();
    }

    int size() {
        return sizeof(width) + sizeof(height) + sizeof(ApolloPoint) * width * height;
    }

    bool Deserialize(const char* buf, size_t size) {
        if(size < (size_t)(2*sizeof(uint32_t))) return false;
        uint32_t w = *((uint32_t*)buf);
        buf += sizeof(uint32_t);
        uint32_t h = *((uint32_t*)buf);
        buf += sizeof(uint32_t);

        size -= 2*sizeof(uint32_t);
        if(size != w*h*sizeof(ApolloPoint)) {
            return false;
        }

        width = w;
        height = h;
        points.resize(w*h);

        memcpy(points.data(), buf, size);
        return true;
    }

    bool Serialize(std::vector<char>& buf) {
        buf.resize(size());
        char* ptr = buf.data();
        memcpy(ptr, &width, sizeof(width));
        ptr += sizeof(width);
        memcpy(ptr, &height, sizeof(height));
        ptr += sizeof(height);

        memcpy(ptr, points.data(), width * height * sizeof(ApolloPoint));
        return true;
    }

    uint32_t width;
    uint32_t height;
    std::vector<ApolloPoint> points;
};

#endif  //APOLLO_POINT_CLOUD_H_
