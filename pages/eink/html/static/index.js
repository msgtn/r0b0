document.querySelector('form').addEventListener('submit', function(e) {
    e.preventDefault();
    
    var fileInput = document.getElementById('file');
    var file = fileInput.files[0];
    
    if (file) {
        console.log("Received file");
        var reader = new FileReader();
        
        reader.onload = function(e) {
            var contents = e.target.result;
            console.log('File contents:', contents);
            let body = {
                event: "file_upload",
                contents: contents
            };
            socket.emit("file_upload", body);
            console.log('emitted');
            // Perform further processing with the file contents
        };
        
        reader.readAsText(file);
    }
});