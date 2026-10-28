/*
 * SPDX-FileCopyrightText: 2025 The LineageOS Project
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "AHAL_UltrasoundProximity_QTI"

#include <android-base/logging.h>
#include <cutils/properties.h>
#include <cutils/sockets.h>
#include <dlfcn.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

#include <extensions/UltrasoundProximity.h>
#include "PalApi.h"
#include "us_detect_api.h"

namespace qti::audio::core {

namespace {

constexpr int kMaxClients = 3;
constexpr int kMaxEvents = 5;
constexpr int kMessageLength = 72;
constexpr const char* kUltrasoundNotifyLibrary = "libultrasound_notify.so";

void (*ultraSoundEventPut)(int event) = nullptr;

std::mutex gLock;
pal_stream_handle_t* gStream = nullptr;
int gInstanceCount = 0;

int32_t handleUpdEvent(pal_stream_handle_t* /*handle*/, uint32_t eventId, uint32_t* eventData,
                       uint32_t /*eventSize*/, uint64_t /*cookie*/) {
    if (eventId != EVENT_ID_GENERIC_US_DETECTION || !eventData) return 0;

    switch (*eventData) {
        case US_DETECT_NEAR:
            LOG(INFO) << "MIUS: near event received";
            ultraSoundEventPut(0);
            break;
        case US_DETECT_FAR:
            LOG(INFO) << "MIUS: far event received";
            ultraSoundEventPut(1);
            break;
        default:
            LOG(INFO) << "MIUS: invalid event " << *eventData;
            break;
    }
    return 0;
}

void startStream() {
    std::lock_guard lock(gLock);

    if (gInstanceCount++ > 0) return;

    pal_stream_attributes attributes{};
    attributes.type = PAL_STREAM_ULTRASOUND;
    attributes.direction = PAL_AUDIO_INPUT_OUTPUT;

    // The device attributes are set by PAL from the backend in use.
    pal_device devices[2]{};
    devices[0].id = PAL_DEVICE_OUT_ULTRASOUND;
    devices[0].config.sample_rate = 96000;
    devices[1].id = PAL_DEVICE_IN_ULTRASOUND_MIC;
    devices[1].config.sample_rate = 96000;

    int32_t status = pal_stream_open(&attributes, 2, devices, 0, nullptr,
                                     (pal_stream_callback)&handleUpdEvent, 0, &gStream);
    if (status) {
        LOG(ERROR) << "MIUS: failed to open the ultrasound stream " << status;
        gStream = nullptr;
        gInstanceCount--;
        return;
    }

    auto* payload = (pal_param_payload*)calloc(
            1, sizeof(pal_param_payload) + sizeof(pal_param_upd_event_detection_t));
    if (payload) {
        pal_param_upd_event_detection_t registerEvents{};
        registerEvents.register_status = 1;
        payload->payload_size = sizeof(registerEvents);
        memcpy(payload->payload, &registerEvents, sizeof(registerEvents));
        status = pal_stream_set_param(gStream, PAL_PARAM_ID_UPD_REGISTER_FOR_EVENTS, payload);
        free(payload);
    } else {
        status = -ENOMEM;
    }

    if (!status) status = pal_stream_start(gStream);
    if (status) {
        LOG(ERROR) << "MIUS: failed to start the ultrasound stream " << status;
        pal_stream_close(gStream);
        gStream = nullptr;
        gInstanceCount--;
        return;
    }
    LOG(INFO) << "MIUS: ultrasound stream started";
}

void stopStream() {
    usleep(property_get_int32("vendor.audio.ultrasound.usync", 1000) * 1000);

    std::lock_guard lock(gLock);

    if (gInstanceCount == 0) return;
    if (--gInstanceCount > 0 || !gStream) return;

    if (pal_stream_stop(gStream)) LOG(ERROR) << "MIUS: pal_stream_stop failed";
    if (pal_stream_close(gStream)) LOG(ERROR) << "MIUS: pal_stream_close failed";
    gStream = nullptr;
    LOG(INFO) << "MIUS: ultrasound stream stopped";
}

void closeClient(int epollFd, int fd, int* clientCount) {
    epoll_ctl(epollFd, EPOLL_CTL_DEL, fd, nullptr);
    close(fd);
    (*clientCount)--;
}

void connectionThread(int controlFd, int epollFd) {
    int clientCount = 0;
    epoll_event events[kMaxEvents];
    char buffer[kMessageLength];

    while (true) {
        int numEvents = epoll_wait(epollFd, events, kMaxEvents, -1);
        if (numEvents < 0) {
            if (errno == EINTR) continue;
            PLOG(ERROR) << "MIUS: epoll_wait failed";
            return;
        }

        for (int i = 0; i < numEvents; i++) {
            int fd = events[i].data.fd;
            uint32_t flags = events[i].events;

            if (fd == controlFd) {
                int clientFd = accept(controlFd, nullptr, nullptr);
                if (clientFd < 0) {
                    PLOG(ERROR) << "MIUS: accept failed";
                } else if (clientCount >= kMaxClients) {
                    LOG(ERROR) << "MIUS: client connections full";
                    close(clientFd);
                } else {
                    epoll_event clientEvent{};
                    clientEvent.events = EPOLLIN | EPOLLRDHUP;
                    clientEvent.data.fd = clientFd;
                    epoll_ctl(epollFd, EPOLL_CTL_ADD, clientFd, &clientEvent);
                    clientCount++;
                    LOG(INFO) << "MIUS: accepted client " << clientFd << ", total "
                              << clientCount;
                }
                continue;
            }

            if (flags & EPOLLIN) {
                ssize_t bytes = read(fd, buffer, sizeof(buffer));
                if (bytes <= 0) {
                    closeClient(epollFd, fd, &clientCount);
                    continue;
                }
                if (bytes != kMessageLength) {
                    LOG(ERROR) << "MIUS: message too short " << bytes;
                } else if (buffer[0] == '*') {
                    int cmd = buffer[8];
                    LOG(INFO) << "MIUS: ultrasound-proximity=" << cmd;
                    if (cmd == 0 || cmd == 1) {
                        ultrasoundProximityEnable(cmd == 1);
                    } else {
                        LOG(ERROR) << "MIUS: unknown payload " << cmd;
                    }
                } else {
                    LOG(ERROR) << "MIUS: unknown command code";
                }
                buffer[0] = 1;
                write(fd, buffer, 2);
            }
            if (flags & (EPOLLHUP | EPOLLRDHUP | EPOLLERR)) {
                closeClient(epollFd, fd, &clientCount);
            }
        }
    }
}

} // namespace

void ultrasoundProximityStart() {
    static std::once_flag started;
    std::call_once(started, [] {
        int controlFd = android_get_control_socket("audio_us_socket_0");
        if (controlFd < 0) return;

        void* handle = dlopen(kUltrasoundNotifyLibrary, RTLD_NOW);
        if (!handle) {
            LOG(ERROR) << "MIUS: failed to load " << kUltrasoundNotifyLibrary << ": "
                       << dlerror();
            return;
        }
        ultraSoundEventPut = (void (*)(int))dlsym(handle, "ultraSoundEventPut");
        if (!ultraSoundEventPut) {
            LOG(ERROR) << "MIUS: ultraSoundEventPut not found: " << dlerror();
            dlclose(handle);
            return;
        }

        if (listen(controlFd, 1) < 0) {
            PLOG(ERROR) << "MIUS: listen failed";
            return;
        }

        int epollFd = epoll_create1(EPOLL_CLOEXEC);
        if (epollFd < 0) {
            PLOG(ERROR) << "MIUS: epoll_create failed";
            return;
        }

        epoll_event event{};
        event.events = EPOLLIN;
        event.data.fd = controlFd;
        if (epoll_ctl(epollFd, EPOLL_CTL_ADD, controlFd, &event) < 0) {
            PLOG(ERROR) << "MIUS: epoll_ctl failed";
            close(epollFd);
            return;
        }

        std::thread(connectionThread, controlFd, epollFd).detach();
        LOG(INFO) << "MIUS: ultrasound proximity connection thread started";
    });
}

void ultrasoundProximityEnable(bool enable) {
    std::thread(enable ? startStream : stopStream).detach();
}

} // namespace qti::audio::core
