#ifndef HOVER_PLUGINS_TIDE_PROGRESS_HH_
#define HOVER_PLUGINS_TIDE_PROGRESS_HH_

#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

namespace hover
{
/// Simulation-time tide clock controlled by /scenario_event.
class TideProgress
{
  public: double Advance(double _simTime, bool _paused)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->initialized)
    {
      this->lastSimTime = _simTime;
      this->initialized = true;
    }

    const double step = std::max(0.0, _simTime - this->lastSimTime);
    this->lastSimTime = _simTime;
    for (const auto &event : this->pendingEvents)
      this->Apply(event);
    this->pendingEvents.clear();

    if (!_paused && !this->held && !this->stopped)
      this->elapsed += step;
    return this->elapsed;
  }

  public: void QueueEvent(const std::string &_event)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingEvents.push_back(_event);
  }

  private: void Apply(const std::string &_event)
  {
    if (_event == "tide_rise" || _event == "rise" || _event == "start_tide")
    {
      this->elapsed = 0.0;
      this->held = false;
      this->stopped = false;
    }
    else if (_event == "tide_reset" || _event == "reset")
    {
      this->elapsed = 0.0;
      this->held = false;
      this->stopped = true;
    }
    else if (_event == "tide_hold" || _event == "hold")
      this->held = true;
    else if (_event == "tide_resume" || _event == "resume")
      this->held = false;
  }

  private: std::mutex mutex;
  private: std::vector<std::string> pendingEvents;
  private: double elapsed{0.0};
  private: double lastSimTime{0.0};
  private: bool initialized{false};
  private: bool held{false};
  private: bool stopped{false};
};
}  // namespace hover

#endif
