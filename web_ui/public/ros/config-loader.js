// public/ros/config-loader.js
fetch("/config/config.json")
    .then((res) => res.json())
    .then((config) => {
        window.AppConfig = config;
        console.log("✅ AppConfig loaded", window.AppConfig);
        window.dispatchEvent(new Event("appConfigReady")); // optional signal
    })
    .catch((err) => {
        console.error("❌ Failed to load AppConfig:", err);
    });
