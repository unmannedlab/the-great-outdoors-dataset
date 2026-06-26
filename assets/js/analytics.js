(function () {
  function normalizeStats(payload) {
    var rows = Array.isArray(payload) ? payload : payload && (payload.countries || payload.regions || payload.locations);
    if (!Array.isArray(rows)) {
      return [];
    }

    return rows
      .map(function (item) {
        var label = item.label || item.country || item.region || item.name || item.code;
        var value = item.value || item.visits || item.pageviews || item.count;
        return {
          label: label ? String(label) : "",
          value: value === 0 || value ? String(value) : "",
          detail: item.detail || item.percent || item.share || ""
        };
      })
      .filter(function (item) {
        return item.label && item.value;
      });
  }

  function renderStats(list, status, stats, limit) {
    if (!stats.length) {
      if (status) {
        status.textContent = "No public aggregate location statistics are available yet.";
      }
      return;
    }

    list.innerHTML = "";
    stats.slice(0, limit).forEach(function (item) {
      var li = document.createElement("li");
      var label = document.createElement("span");
      var value = document.createElement("strong");
      label.textContent = item.label;
      value.textContent = item.value;
      li.appendChild(label);
      li.appendChild(value);
      if (item.detail) {
        var detail = document.createElement("small");
        detail.textContent = item.detail;
        li.appendChild(detail);
      }
      list.appendChild(li);
    });

    if (status) {
      status.textContent = "Showing public aggregate location statistics only.";
    }
  }

  function copyVisitorCounters() {
    var targets = document.querySelectorAll("[data-counter-copy]");
    if (!targets.length) {
      return;
    }

    var attempts = 0;
    var timer = window.setInterval(function () {
      attempts += 1;
      targets.forEach(function (target) {
        var source = document.getElementById("busuanzi_value_" + target.getAttribute("data-counter-copy"));
        if (source && source.textContent && source.textContent !== "...") {
          target.textContent = source.textContent;
        }
      });

      if (attempts >= 20) {
        window.clearInterval(timer);
      }
    }, 500);
  }

  document.addEventListener("DOMContentLoaded", function () {
    copyVisitorCounters();

    var section = document.querySelector("[data-analytics-section]");
    if (!section) {
      return;
    }

    var url = section.getAttribute("data-location-stats-url");
    var list = section.querySelector("[data-analytics-locations]");
    var status = section.querySelector("[data-analytics-status]");
    var limit = parseInt(section.getAttribute("data-location-limit") || "6", 10);

    if (!url || !list) {
      return;
    }

    fetch(url, { cache: "no-store" })
      .then(function (response) {
        if (!response.ok) {
          throw new Error("Location statistics request failed");
        }
        return response.json();
      })
      .then(function (payload) {
        renderStats(list, status, normalizeStats(payload), limit);
      })
      .catch(function () {
        if (status) {
          status.textContent = "Could not load public aggregate location statistics.";
        }
      });
  });
})();
