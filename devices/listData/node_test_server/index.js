const express = require("express")

const app = express()

const router = require("express").Router()

app.use(express.json())

const port = 3000

app.post('/teste',(req,res)=>{
    data = req.body
    console.log(data)
    
    // for (const info_data of data) {
    //   console.log(info_data); // Prints each fruit
    // }
    res.json({message:"Sucesso"}).status(200)
})

app.listen(port, '0.0.0.0', ()=>{
    console.log(`Server run in port ${port}`)
})