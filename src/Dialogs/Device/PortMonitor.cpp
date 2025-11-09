// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "PortMonitor.hpp"
#include "Dialogs/Message.hpp"
#include "Look/Look.hpp"
#include "ui/control/TerminalWindow.hpp"
#include "Widget/WindowWidget.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Device/Descriptor.hpp"
#include "util/StaticFifoBuffer.hxx"
#include "Language/Language.hpp"
#include "Operation/MessageOperationEnvironment.hpp"
#include "ui/event/DelayedNotify.hpp"
#include "thread/Mutex.hxx"
#include "UIGlobals.hpp"

/**
 * A bridge between DataHandler and TerminalWindow: copy all data
 * received from the Port to the TerminalWindow.
 */
class PortTerminalBridge final : public DataHandler {
  TerminalWindow &terminal;
  Mutex mutex;
  StaticFifoBuffer<std::byte, 1024> buffer;
  bool hexOutput = false;

  UI::DelayedNotify notify{
    std::chrono::milliseconds(100),
    [this]{ OnNotification(); },
  };

public:
  PortTerminalBridge(TerminalWindow &_terminal)
    :terminal(_terminal) {}

  PortTerminalBridge(TerminalWindow &_terminal, DeviceDescriptor &_device)
    :terminal(_terminal) {
      hexOutput = _device.GetConfig().port_type == DeviceConfig::PortType::CAN_INTERFACE;
  }
  virtual ~PortTerminalBridge() {}

  bool DataReceived(std::span<const std::byte> s) noexcept {
    {
      const std::lock_guard lock{mutex};
      buffer.Shift();
      auto range = buffer.Write();

      if (range.empty())
        return true; // buffer full; drop

      if (hexOutput) {
        // How many input bytes can we encode given the available space?
        // Need 2 chars per byte, plus CRLF
        const std::size_t cap = range.size();
        if (cap < 2)
          return true; // no space even for CRLF

        const std::size_t max_in = (cap - 2) / 2;
        const std::size_t in_n = std::min(s.size(), max_in);
        auto out_it = range.begin();

        static constexpr char tohex[] = "0123456789ABCDEF";
        for (std::size_t i = 0; i < in_n; ++i) {
          const unsigned char v = static_cast<unsigned char>(s[i]);
          *out_it++ = static_cast<std::byte>(tohex[(v >> 4) & 0xF]);
          *out_it++ = static_cast<std::byte>(tohex[v & 0xF]);
        }
        // Append CRLF
        *out_it++ = static_cast<std::byte>('\r');
        *out_it++ = static_cast<std::byte>('\n');

        const std::size_t out_len = (in_n * 2) + 2;
        buffer.Append(out_len);

      } else {
        const std::size_t nbytes = std::min(s.size(), range.size());
        std::copy_n(s.begin(), nbytes, range.begin());
        buffer.Append(nbytes);
      }
    }

    notify.SendNotification();
    return true;
  }

private:
  void OnNotification() noexcept {
    while (true) {
      std::array<std::byte, 64> data;
      size_t length;

      {
        const std::lock_guard lock{mutex};
        auto range = buffer.Read();
        if (range.empty())
          break;

        length = std::min(data.size(), range.size());
        std::copy_n(range.begin(), length, data.begin());
        buffer.Consume(length);
      }

      terminal.Write((const char *)data.data(), length);
    }
  }

  char *bin2hex(const unsigned char *bin, size_t *len) {
      static const char *const tohex = "0123456789ABCDEF";
      char *out;
      size_t i;
      if (bin == NULL || len == 0)
          return NULL;
      out = static_cast<char *>(malloc((*len * 2) + 2));
      for (i = 0; i < *len; i++) {
          out[i * 2] = tohex[bin[i] >> 4];
          out[(i * 2) + 1] = tohex[bin[i] & 0x0F];
      }
      out[*len * 2] = '\r';
      out[(*len * 2) + 1] = '\n';
      *len = (*len * 2) + 2;
      return out;
  }

};

class PortMonitorWidget final : public WindowWidget {
  DeviceDescriptor &device;
  const TerminalLook &look;
  std::unique_ptr<PortTerminalBridge> bridge;

  Button *pause_button;
  bool paused;

public:
  PortMonitorWidget(DeviceDescriptor &_device,
                    const TerminalLook &_look) noexcept
    :device(_device), look(_look), paused(false) {}

  void CreateButtons(WidgetDialog &dialog);

  void Clear() {
    auto &terminal = (TerminalWindow &)GetWindow();
    terminal.Clear();
  }

  void Reconnect();
  void TogglePause();

  /* virtual methods from class Widget */

  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override {
    WindowStyle style;
    style.Hide();

    auto w = std::make_unique<TerminalWindow>(look);
    w->Create(parent, rc, style);

    bridge = std::make_unique<PortTerminalBridge>(*w, device);
    device.SetMonitor(bridge.get());

    SetWindow(std::move(w));
  }

  void Unprepare() noexcept override {
    device.SetMonitor(nullptr);
  }
};

void
PortMonitorWidget::CreateButtons(WidgetDialog &dialog)
{
  dialog.AddButton(_("Clear"), [this](){ Clear(); });
  dialog.AddButton(_("Reconnect"), [this](){ Reconnect(); });
  pause_button = dialog.AddButton(_("Pause"), [this](){ TogglePause(); });
}

void
PortMonitorWidget::Reconnect()
{
  if (device.IsBorrowed()) {
    ShowMessageBox(_("Device is occupied"), _("Reconnect"),
                   MB_OK | MB_ICONERROR);
    return;
  }

  /* this OperationEnvironment instance must be persistent, because
     DeviceDescriptor::Open() is asynchronous */
  static MessageOperationEnvironment env;
  device.Reopen(env);
}

void
PortMonitorWidget::TogglePause()
{
  paused = !paused;

  if (paused) {
    pause_button->SetCaption(_("Resume"));
    device.SetMonitor(nullptr);
  } else {
    pause_button->SetCaption(_("Pause"));
    device.SetMonitor(bridge.get());
  }
}

void
ShowPortMonitor(DeviceDescriptor &device)
{
  const Look &look = UIGlobals::GetLook();

  std::array<TCHAR, 64> buffer;
  StaticString<128> caption;
  caption.Format(_T("%s: %s"), _("Port monitor"),
                 device.GetConfig().GetPortName(buffer.data(), buffer.size()));

  TWidgetDialog<PortMonitorWidget>
    dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
           look.dialog, caption);
  dialog.AddButton(_("Close"), mrOK);
  dialog.SetWidget(device, look.terminal);
  dialog.GetWidget().CreateButtons(dialog);

  dialog.ShowModal();
}
