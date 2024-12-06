Navigate to backend directory
```bash
cd backend/app
```
Create and activate virtual environment
```bash
python3 -m venv venv
source venv/bin/activate
```
Install dependencies
```bash
pip install -r requirements.txt
```

Edit main.py - Replace YOUR_IP with your machine's IP address
Line 15: server = await serve(partial(manager.handler), "YOUR_IP", 8765)

Start the backend server

```bash
uvicorn main:app --host "YOUR_IP" --port 8000
```
Navigate to frontend directory
```bash
cd frontend
```
Install dependencies
```bash
npm install
```
Edit WebSocketContext.tsx - Replace YOUR_IP with your machine's IP address
Line 50: const ws = new WebSocket(`ws://YOUR_IP:8765`);

Start the frontend application
```bash
npm start
```
