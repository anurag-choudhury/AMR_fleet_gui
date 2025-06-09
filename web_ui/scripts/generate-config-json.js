const fs = require("fs");
const path = require("path");

const inputPath = path.join(__dirname, "../src/shared/constants/index.js");
const outputPath = path.join(__dirname, "../public/config/config.json");

const fileContent = fs.readFileSync(inputPath, "utf8");

// Extract the object using a regex hack
const match = fileContent.match(/export const AppConfig\s*=\s*({[\s\S]*?});/);

if (!match) {
    console.error("❌ Could not find AppConfig export in index.js");
    process.exit(1);
}

try {
    const jsObjectCode = match[1];

    // Safely evaluate the object (dangerous in general, but fine here in dev)
    const config = eval('(' + jsObjectCode + ')');

    fs.mkdirSync(path.dirname(outputPath), { recursive: true });
    fs.writeFileSync(outputPath, JSON.stringify(config, null, 2));
    console.log("✅ Config JSON generated at public/config/config.json");
} catch (err) {
    console.error("❌ Failed to parse AppConfig:", err);
}
