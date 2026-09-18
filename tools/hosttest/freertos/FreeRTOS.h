#pragma once
#include <deque>
#include <vector>
#include <cstring>
#include <cstdint>
typedef void* SemaphoreHandle_t;
struct QMock { size_t item; size_t cap; std::deque<std::vector<uint8_t>> q; };
typedef QMock* QueueHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(x) (x)
extern bool g_mtx_locked;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (void*)1; }
inline int xSemaphoreTake(SemaphoreHandle_t, uint32_t) { if (g_mtx_locked) return pdFALSE; g_mtx_locked = true; return pdTRUE; }
inline void xSemaphoreGive(SemaphoreHandle_t) { g_mtx_locked = false; }
inline QueueHandle_t xQueueCreate(size_t n, size_t item) { return new QMock{item, n, {}}; }
inline int xQueueSend(QueueHandle_t q, const void* p, uint32_t) { if (q->q.size() >= q->cap) return pdFALSE; q->q.emplace_back((const uint8_t*)p, (const uint8_t*)p + q->item); return pdTRUE; }
inline int xQueueReceive(QueueHandle_t q, void* p, uint32_t) { if (q->q.empty()) return pdFALSE; memcpy(p, q->q.front().data(), q->item); q->q.pop_front(); return pdTRUE; }
inline int xTaskCreate(void (*)(void*), const char*, int, void*, int, void*) { return pdTRUE; }
extern uint32_t g_now;
inline void vTaskDelay(uint32_t ms) { g_now += ms; }
