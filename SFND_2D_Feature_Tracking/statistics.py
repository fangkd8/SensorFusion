f = open("MP_7.csv", "r")
pts_mean = [0]*7
idx = 0
name = []
for x in f:
  # print(x)
  s = x.strip()
  s = s.split(", ")
  name.append(s[0])
  for i in range(1, len(s)):
    # print(i)
    pts_mean[idx] += int(s[i])/10
  idx += 1

print("KeyPoints Number:")
for i in range(len(name)):
  print(name[i], " mean:", pts_mean[i])
print("")

match_mean = [0]*28
name = []  
f = open("MP_8.csv", "r")
idx = 0
for x in f:
  s = x.strip()
  s = s.split(", ")
  name.append(s[0])
  for i in range(1, len(s)):
    # print(idx)
    match_mean[idx] += int(s[i])/9
  idx += 1
print("Matches Number:")
for i in range(len(name)):
  print(name[i], "  mean:", match_mean[i])
print("")

time_mean = [0]*28
name = []  
f = open("MP_9.csv", "r")
idx = 0
for x in f:
  s = x.strip()
  s = s.split(", ")
  name.append(s[0])
  for i in range(1, len(s)):
    # print(idx)
    time_mean[idx] += float(s[i])/9
  idx += 1
print("Mean Time:")
for i in range(len(name)):
  print(name[i], "  mean:", time_mean[i])
print("")

print("10 fastest combination")
minTime = sorted(range(len(time_mean)), key=lambda k: time_mean[k])
for i in range(10):
  id = minTime[i]
  print(name[id], time_mean[id])
print("")
print("10 most keypoints tracking")
maxTracks = sorted(range(len(time_mean)), key=lambda k: time_mean[k])[::-1]
for i in range(10):
  id = maxTracks[i]
  print(name[id], match_mean[id])
