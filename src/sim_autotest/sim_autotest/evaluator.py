import rclpy, json, math, csv, time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray

def match(gt, tracks, gate=1.0):
    # greedy nearest neighbor
    used=set(); pairs=[]
    for i,g in enumerate(gt):
        best=-1; bestd=1e9
        for j,t in enumerate(tracks):
            if j in used: continue
            d=math.hypot(g[0]-t[0], g[1]-t[1])
            if d<bestd: bestd=d; best=j
        if best>=0 and bestd<=gate:
            used.add(best); pairs.append((i,best,bestd))
    return pairs

class Evaluator(Node):
    def __init__(self):
        super().__init__('evaluator')
        self.duration = float(self.declare_parameter('duration', 60.0).value)
        self.start = time.time()
        self.gt=[]; self.tracks=[]
        self.sub_gt = self.create_subscription(PoseArray, '/gt/poses', self.on_gt, 10)
        self.sub_tr = self.create_subscription(String, '/tracks/states', self.on_tr, 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.mae_sum=0.0; self.mae_count=0; self.idsw=0
        self.last_pairs=None
        self.outfile='results/metrics.csv'
        self.get_logger().info("Evaluator collecting stats...")

    def on_gt(self, msg: PoseArray):
        self.gt = [(p.position.x, p.position.y) for p in msg.poses]

    def on_tr(self, msg: String):
        data = json.loads(msg.data) if msg.data else {}
        pts=[]
        for t in data.get('tracks', []):
            x=t['x']; pts.append((x[0], x[1]))
        self.tracks = pts

    def on_timer(self):
        if time.time() - self.start >= self.duration:
            try:
                with open(self.outfile,'w',newline='') as f:
                    w=csv.writer(f)
                    w.writerow(['mae', 'pairs', 'idsw_estimate'])
                    mae = self.mae_sum/max(1,self.mae_count)
                    w.writerow([f"{mae:.3f}", self.mae_count, self.idsw])
                self.get_logger().info(f"Wrote {self.outfile}")
            except Exception as e:
                self.get_logger().error(str(e))
            rclpy.shutdown()

        # increment metrics
        pairs = match(self.gt, self.tracks, gate=1.5)
        if pairs:
            self.mae_sum += sum(d for _,_,d in pairs)/len(pairs)
            self.mae_count += 1
            # crude IDSW proxy: if pairing pattern changes drastically frame-to-frame
            if self.last_pairs is not None and len(self.last_pairs)==len(pairs):
                if sum(1 for a,b,_ in pairs if a==self.last_pairs[0][0] and b!=self.last_pairs[0][1])>0:
                    self.idsw += 1
        self.last_pairs = pairs

def main():
    rclpy.init(); n=Evaluator(); rclpy.spin(n)

if __name__=='__main__':
    main()
