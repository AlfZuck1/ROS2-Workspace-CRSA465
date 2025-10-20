from http.server import SimpleHTTPRequestHandler, HTTPServer

class CORSRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

if __name__ == '__main__':
    server_address = ('', 8081)  # puerto 8081
    httpd = HTTPServer(server_address, CORSRequestHandler)
    print("Servidor HTTP con CORS en puerto 8081")
    httpd.serve_forever()