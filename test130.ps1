$serial=[System.IO.Ports.SerialPort]::new('COM8',115200,[System.IO.Ports.Parity]::None,8,[System.IO.Ports.StopBits]::One)
$log=[System.Text.StringBuilder]::new()
try {
 $serial.Open()
 $serial.Write("`r`nSTOP`r`nSTREAM 0`r`n")
 Start-Sleep -Milliseconds 250
 [void]$log.Append($serial.ReadExisting())
 foreach($axis in @(1,2)) {
  $serial.Write("STATUS`r`n"); Start-Sleep -Milliseconds 200
  $pre=$serial.ReadExisting(); [void]$log.Append($pre); Write-Output $pre
  if($pre -notmatch "AXIS $axis distance_mm=([0-9.]+) valid=1 .*state=0 "){throw "Axis $axis not ready"}
  $distance=[double]$Matches[1]
  if($distance -lt 129.5 -or $distance -gt 275.5){throw "Axis $axis outside configured travel"}
  Write-Output "SEND GRIP $axis"
  [void]$log.AppendLine("SEND GRIP $axis")
  $serial.Write("GRIP $axis`r`n")
  $finished=$false
  for($j=0;$j -lt 185;$j++) {
   Start-Sleep -Milliseconds 900
   $serial.Write("STATUS`r`n"); Start-Sleep -Milliseconds 100
   $chunk=$serial.ReadExisting(); [void]$log.Append($chunk)
   if($j%5 -eq 0){Write-Output $chunk}
   if($chunk -match "AXIS $axis .*state=3 "){Write-Output $chunk;throw "Axis $axis fault"}
   if($chunk -match "AXIS $axis distance_mm=([0-9.]+) valid=1 .*state=0 "){
    $actual=[double]$Matches[1]
    Write-Output "ARRIVED axis=$axis distance=$actual elapsed_s=$($j+1)"
    if($actual -lt 129.0 -or $actual -gt 130.5){throw "Arrival outside expected band"}
    $finished=$true;break
   }
  }
  if(-not $finished){throw "Host test timeout"}
  for($k=0;$k -lt 3;$k++){$serial.Write("STATUS`r`n");Start-Sleep -Milliseconds 500;$chunk=$serial.ReadExisting();[void]$log.Append($chunk);Write-Output $chunk}
 }
} finally {
 if($serial.IsOpen){$serial.Write("STOP`r`nSTREAM 1`r`n");Start-Sleep -Milliseconds 150;Write-Output $serial.ReadExisting();$serial.Close()}
 $serial.Dispose()
 $log.ToString()|Add-Content -LiteralPath C:/D/AIRPORT/com8-endpoint130-test.txt -Encoding utf8
}
