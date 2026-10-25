#pragma once

#include <Arduino.h>
#include <new>
#include <time.h>
#include <atomic>
#include "RadioBrowserProvider.h"
#include "AkashvaniProvider.h"

namespace RadioCatalog {
class CatalogService {
 public:
  bool begin() {
    requests_ = xQueueCreate(1, sizeof(Request));
    results_ = xQueueCreate(1, sizeof(Result*));
    started_ = xQueueCreate(1, sizeof(uint32_t));
    if (!requests_ || !results_ || !started_) return false;
    return xTaskCreate(taskEntry, "catalog", 12288, this, 1, &task_) == pdPASS;
  }
  bool request(const Request& request) {
    desiredGeneration_.store(request.generation);
    return requests_ && xQueueOverwrite(requests_, &request) == pdTRUE;
  }
  void cancel(uint32_t generation) {
    desiredGeneration_.compare_exchange_strong(generation, 0);
  }
  // Results have exactly one owner: worker until received, then caller.
  bool poll(Result*& result) { return results_ && xQueueReceive(results_, &result, 0) == pdTRUE; }
  bool pollStarted(uint32_t& generation) { return started_ && xQueueReceive(started_, &generation, 0) == pdTRUE; }

 private:
  QueueHandle_t requests_ = nullptr, results_ = nullptr, started_ = nullptr;
  TaskHandle_t task_ = nullptr;
  RadioBrowserProvider radioBrowser_;
  AkashvaniProvider akashvani_;
  std::atomic<uint32_t> desiredGeneration_{0};
  struct ActiveRequest { CatalogService* service; uint32_t generation; };
  static void taskEntry(void* self) { static_cast<CatalogService*>(self)->run(); }
  static bool superseded(void* self) {
    const ActiveRequest& active = *static_cast<ActiveRequest*>(self);
    return active.service->desiredGeneration_.load() != active.generation;
  }
  void deliver(Result* result) {
    Result* stale = nullptr;
    if (xQueueReceive(results_, &stale, 0) == pdTRUE) delete stale;
    if (xQueueSend(results_, &result, 0) != pdTRUE) delete result;
  }
  void run() {
    Request request;
    for (;;) {
      if (xQueueReceive(requests_, &request, portMAX_DELAY) != pdTRUE) continue;
      if(desiredGeneration_.load() != request.generation) continue;
      xQueueOverwrite(started_, &request.generation);
      Result* result = new (std::nothrow) Result;
      if (!result) { Serial.println("catalog: no memory for result"); continue; }
      result->request = request;
      ActiveRequest active{this, request.generation};
      RadioNetwork::ReadCancellation cancellation(superseded, &active);
      const bool isAkashvani = request.filter.source == RadioModel::Source::Akashvani;
      const bool local = isAkashvani && akashvani_.canServe(request);
      if (!local) {
        if (WiFi.status() != WL_CONNECTED) {
          copyField(result->message, sizeof(result->message), "Wi-Fi offline; previous results kept");
          deliver(result); continue;
        }
        const uint32_t start = millis();
        while (time(nullptr) < 1700000000 && millis() - start < 10000 && !cancellation.requested())
          vTaskDelay(pdMS_TO_TICKS(100));
        if (cancellation.requested()) { delete result; continue; }
        if (time(nullptr) < 1700000000) {
          copyField(result->message, sizeof(result->message), "Waiting for network clock (TLS)");
          deliver(result); continue;
        }
      }
      const bool success = isAkashvani ? akashvani_.fetch(*result, cancellation) : radioBrowser_.fetch(*result, cancellation);
      if (cancellation.requested()) { delete result; continue; }
      if (success) {
        result->status = result->count ? ResultStatus::Success : ResultStatus::Empty;
        copyField(result->message, sizeof(result->message), result->count ? "Stations ready" : "No matching stations");
      }
      deliver(result);
    }
  }
};
} // namespace RadioCatalog
