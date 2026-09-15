//go:build ignore

// Dev tool: prints rows out of a Parquet file produced by encode.go (or by
// the real sync tool later), so you can eyeball it without installing
// anything else. Run with `go run view.go [flags]`.
package main

import (
	"flag"
	"fmt"
	"log"
	"os"

	"github.com/parquet-go/parquet-go"

	"aero-engine-dt/sync/internal/parquetenc"
)

func main() {
	inPath := flag.String("in", "out.parquet", "Parquet file to read")
	n := flag.Int("n", 5, "rows to print from the start and end each (0 = all)")
	flag.Parse()

	f, err := os.Open(*inPath)
	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()

	reader := parquet.NewGenericReader[parquetenc.Row](f)
	defer reader.Close()

	total := int(reader.NumRows())
	rows := make([]parquetenc.Row, total)
	read, err := reader.Read(rows)
	if err != nil && read != total {
		log.Fatalf("reading %s: %v (read %d of %d rows)", *inPath, err, read, total)
	}

	fmt.Printf("%s: %d rows\n", *inPath, total)
	print := func(i int) {
		r := rows[i]
		fmt.Printf("  [%d] t=%.4g throttle=%.3g rpm=%.6g cht_c=%.6g egt_c=%.6g oil_c=%.6g\n",
			i, r.T, r.Throttle, r.RPM, r.CHTC, r.EGTC, r.OilC)
	}

	if *n <= 0 || total <= 2*(*n) {
		for i := range rows {
			print(i)
		}
		return
	}
	for i := 0; i < *n; i++ {
		print(i)
	}
	fmt.Println("  ...")
	for i := total - *n; i < total; i++ {
		print(i)
	}
}
