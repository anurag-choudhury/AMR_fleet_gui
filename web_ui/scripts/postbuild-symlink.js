const fs = require("fs");
const path = require("path");

const buildPath = path.resolve(__dirname, "../build");
const targetPath = path.resolve(__dirname, "../../ui_package/src");

const links = [
    { src: path.join(buildPath, "static"), dest: path.join(targetPath, "static") },
    { src: path.join(buildPath, "ros"), dest: path.join(targetPath, "ros") },
    { src: path.join(buildPath, "config"), dest: path.join(targetPath, "config") },
    { src: path.join(buildPath, "index.html"), dest: path.join(targetPath, "templates/index.html") },
];

// Ensure templates dir exists
fs.mkdirSync(path.join(targetPath, "templates"), { recursive: true });

for (const { src, dest } of links) {
    try {
        const destExists = fs.existsSync(dest);

        if (destExists) {
            const stat = fs.lstatSync(dest);
            if (stat.isSymbolicLink()) {
                fs.unlinkSync(dest);
            } else if (stat.isDirectory()) {
                // Don't touch real directories
                console.warn(`⚠️  Skipping ${dest} (real directory, not a symlink)`);
                continue;
            } else {
                fs.unlinkSync(dest); // remove regular file
            }
        }

        const isDir = fs.lstatSync(src).isDirectory();
        fs.symlinkSync(src, dest, isDir ? "dir" : "file");
        console.log(`✅ Linked ${dest} -> ${src}`);
    } catch (err) {
        console.error(`❌ Failed to link ${dest}:`, err.message);
    }
}
